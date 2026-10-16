//! RCS roll fans M1-M4.
//!
//! # How the hardware works
//! All four fans share one 4.4 V supply, but each has its own on/off switch
//! (a transistor) driven by one pin: M1 = PD3, M2 = PD2, M3 = PD1, M4 = PD0.
//! Pin high = fan on. Every pin has a pull-down resistor, so a dead or
//! resetting MCU leaves all fans off.
//!
//! Partial power comes from flickering a pin on and off quickly (PWM). The
//! "duty" is the fraction of each cycle the fan is on. Duty is NOT speed or
//! thrust: the fan's real response has to be measured on the bench.
//!
//! PD0-PD3 have no built-in timer PWM hardware, so the flicker is made in
//! software. This module computes *what* to write to the pins; the hardware
//! layer decides *when* (see the bottom of this file and `README.md`).
//!
//! # Current readings
//! Each fan's switch has a 50 mOhm current sensor (5 V/A after the amp).
//! Two caveats:
//! - It saturates near 0.66 A. Readings near there mean "at least this".
//! - It only sees current while the switch is on; during the off part of a
//!   cycle the motor's current loops through the flyback diode and bypasses
//!   the sensor. So [`Rcs::update_current`] expects an **average over whole
//!   cycles** (the ADC sampling averaged across a cycle). That average is
//!   supply-path current, not total motor current.

// Fans are four parallel arrays indexed by fan number; plain index loops read
// clearer here than zipped iterators.
#![allow(clippy::needless_range_loop)]

use crate::{config::rcs as cfg, protocol::RcsReport};

const FANS: usize = cfg::FAN_COUNT;

/// What the fan block is currently allowed to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RcsState {
    /// Outputs forced off. Normal "not flying" state.
    Disabled,
    /// Outputs follow [`Rcs::set_duty`].
    Enabled,
    /// Forced off by a fault or abort. Stays off until [`Rcs::clear_safe`].
    Safed,
}

/// Why a duty request was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RcsError {
    /// Fans are disabled or safed. Request ignored, outputs stay off.
    NotEnabled,
    /// NaN, infinite, or outside `0..=1`. Request ignored, previous duty kept.
    Invalid,
}

/// Fan duty bookkeeping, limits and fault handling.
#[derive(Clone, Debug)]
pub struct Rcs {
    state: RcsState,
    /// Last duty the FC asked for (for reporting).
    requested: [f32; FANS],
    /// Duty that will go out at the next cycle boundary.
    pending: [f32; FANS],
    /// Duty currently on the pins.
    active: [f32; FANS],
    /// How long each fan has been over the current limit.
    over_limit_ms: [u64; FANS],
    /// Bit `n` set: fan `n` was shut off for overcurrent.
    faulted: u8,
    /// Last current reading per fan, in amps.
    current_a: [f32; FANS],
}

impl Default for Rcs {
    fn default() -> Self {
        Self::new()
    }
}

impl Rcs {
    /// Starts disabled with every duty at zero.
    pub const fn new() -> Self {
        Self {
            state: RcsState::Disabled,
            requested: [0.0; FANS],
            pending: [0.0; FANS],
            active: [0.0; FANS],
            over_limit_ms: [0; FANS],
            faulted: 0,
            current_a: [0.0; FANS],
        }
    }

    pub fn state(&self) -> RcsState {
        self.state
    }

    /// Allows the fans to follow commands. Does nothing while safed: a
    /// fault has to be acknowledged first with [`Rcs::clear_safe`].
    pub fn enable(&mut self) {
        if self.state == RcsState::Disabled {
            self.state = RcsState::Enabled;
        }
    }

    /// Turns the fans off (normal disarm). Does not leave the safed state.
    pub fn disable(&mut self) {
        if self.state == RcsState::Enabled {
            self.state = RcsState::Disabled;
        }
        self.zero();
    }

    /// Forces every fan off right now and ignores commands until
    /// [`Rcs::clear_safe`]. The hardware layer should also write
    /// [`SAFE_BSRR`] immediately rather than wait for the cycle boundary.
    pub fn safe(&mut self) {
        self.state = RcsState::Safed;
        self.zero();
        self.active = [0.0; FANS];
    }

    /// Acknowledges a fault: safed -> disabled, and clears per-fan
    /// overcurrent faults.
    pub fn clear_safe(&mut self) {
        if self.state == RcsState::Safed {
            self.state = RcsState::Disabled;
        }
        self.faulted = 0;
        self.over_limit_ms = [0; FANS];
    }

    fn zero(&mut self) {
        self.requested = [0.0; FANS];
        self.pending = [0.0; FANS];
    }

    /// Takes a new duty request, `[M1, M2, M3, M4]` in `0..=1`.
    ///
    /// Increases are slew-limited by `MAX_DUTY_SLEW_PER_S`, so a fan cannot
    /// jump from 0 to 100% and sag the shared 4.4 V supply. Decreases happen
    /// immediately, since dropping power is always safe. Faulted fans stay at
    /// zero.
    ///
    /// Returns the duty that will actually be applied.
    pub fn set_duty(&mut self, request: [f32; FANS], dt_s: f32) -> Result<[f32; FANS], RcsError> {
        if request
            .iter()
            .any(|d| !d.is_finite() || !(0.0..=1.0).contains(d))
        {
            return Err(RcsError::Invalid);
        }
        if self.state != RcsState::Enabled {
            return Err(RcsError::NotEnabled);
        }

        self.requested = request;
        let max_step = cfg::MAX_DUTY_SLEW_PER_S * dt_s.max(0.0);
        for fan in 0..FANS {
            let target = if self.is_faulted(fan) {
                0.0
            } else {
                request[fan]
            };
            let from = self.pending[fan];
            self.pending[fan] = if target > from {
                target.min(from + max_step)
            } else {
                target
            };
        }

        Ok(self.pending)
    }

    /// Call at the start of every flicker cycle. Moves the pending duty onto
    /// the pins and returns it, so a cycle never mixes an old and a new duty.
    pub fn latch(&mut self) -> [f32; FANS] {
        self.active = self.pending;
        self.active
    }

    /// Duty currently on the pins.
    pub fn active(&self) -> [f32; FANS] {
        self.active
    }

    fn is_faulted(&self, fan: usize) -> bool {
        self.faulted & (1 << fan) != 0
    }

    /// Feeds in one current reading per fan (amps, averaged over whole
    /// flicker cycles) taken `dt_ms` after the last one.
    ///
    /// A fan that stays at or above `OVERCURRENT_A` for
    /// `OVERCURRENT_DEBOUNCE_MS` is shut off and latched as faulted.
    /// Returns the fans newly faulted by this call, as a bitmask.
    pub fn update_current(&mut self, amps: [f32; FANS], dt_ms: u64) -> u8 {
        self.current_a = amps;
        let mut newly = 0;
        for fan in 0..FANS {
            // A NaN reading is treated as over the limit: a broken
            // measurement is not proof the fan is healthy.
            let over = amps[fan].is_nan() || amps[fan] >= cfg::OVERCURRENT_A;
            self.over_limit_ms[fan] = if over {
                self.over_limit_ms[fan].saturating_add(dt_ms)
            } else {
                0
            };
            if self.over_limit_ms[fan] >= cfg::OVERCURRENT_DEBOUNCE_MS && !self.is_faulted(fan) {
                self.faulted |= 1 << fan;
                self.pending[fan] = 0.0;
                self.active[fan] = 0.0;
                newly |= 1 << fan;
            }
        }
        newly
    }

    /// Bitmask of fans shut off for overcurrent.
    pub fn faulted(&self) -> u8 {
        self.faulted
    }

    /// Builds the fan block of the status message.
    pub fn report(&self) -> RcsReport {
        let mut saturated = 0;
        for (fan, amps) in self.current_a.iter().enumerate() {
            if *amps >= cfg::SENSE_SATURATION_A {
                saturated |= 1 << fan;
            }
        }
        RcsReport {
            requested: self.requested,
            applied: self.active,
            current_a: self.current_a,
            current_saturated: saturated,
            faulted: self.faulted,
        }
    }
}

/// Converts the fan current-sense ADC voltage to amps.
pub fn sense_volts_to_amps(volts: f32) -> f32 {
    volts / cfg::SENSE_V_PER_A
}

// ---------------------------------------------------------------------------
// Waveform generation: turning duties into pin writes.
//
// GPIOD's BSRR register sets and clears pins in one write: bit n sets pin n
// high, bit n+16 sets pin n low. Two ways to drive it:
//
// A) DMA table (no CPU while running): a timer ticks PWM_STEPS times per
//    cycle and each tick DMA copies the next word of `fill_bsrr_table` into
//    GPIOD->BSRR. Double-buffer it and refill the idle half at each cycle
//    boundary. Needs DMA to reach GPIOD; check the STM32H745 reference
//    manual (RM0399) bus matrix.
//
// B) Timer interrupts: at each cycle start write `EdgeSchedule::start`, then
//    schedule a compare interrupt for every entry of `EdgeSchedule::offs`.
//    Only up to five interrupts per cycle, but interrupt delay from Ethernet
//    and other work shows up as timing jitter.
//
// Either way, every write states every fan's pin (set or reset), so one
// missed write corrects itself on the next.
// ---------------------------------------------------------------------------

/// BSRR "set" bit for each fan.
pub const fn set_bit(fan: usize) -> u32 {
    1 << cfg::GPIOD_PIN[fan]
}

/// BSRR "reset" bit for each fan.
pub const fn reset_bit(fan: usize) -> u32 {
    1 << (cfg::GPIOD_PIN[fan] + 16)
}

/// One BSRR write that turns every fan off. Use it at boot, before the pins
/// are switched to outputs, and whenever the EC safes.
pub const SAFE_BSRR: u32 = reset_bit(0) | reset_bit(1) | reset_bit(2) | reset_bit(3);

/// Number of steps of a cycle a fan is on for. Exactly 0 and exactly
/// `PWM_STEPS` at the ends, so 0% never blips on and 100% never blips off.
pub fn duty_to_steps(duty: f32) -> usize {
    if duty.is_nan() || duty <= 0.0 {
        return 0;
    }
    if duty >= 1.0 {
        return cfg::PWM_STEPS;
    }
    // Round to nearest without `f32::round`, which needs std.
    let steps = (duty * cfg::PWM_STEPS as f32 + 0.5) as usize;
    steps.min(cfg::PWM_STEPS)
}

/// Fills one cycle of BSRR words for the DMA approach (A). Word `s` turns a
/// fan on if `s < its on-steps`, otherwise off.
pub fn fill_bsrr_table(duty: &[f32; FANS], table: &mut [u32; cfg::PWM_STEPS]) {
    let on_steps = duty.map(duty_to_steps);
    for (step, word) in table.iter_mut().enumerate() {
        *word = 0;
        for fan in 0..FANS {
            *word |= if step < on_steps[fan] {
                set_bit(fan)
            } else {
                reset_bit(fan)
            };
        }
    }
}

/// The edges in one cycle, for the interrupt approach (B).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdgeSchedule {
    /// Written at step 0: sets fans with duty > 0, resets fans at 0%.
    pub start: u32,
    /// `(step, reset bits)` pairs, sorted by step, one per distinct turn-off
    /// step. Only the first `len` entries are used.
    pub offs: [(u16, u32); FANS],
    pub len: usize,
}

impl EdgeSchedule {
    pub fn new(duty: &[f32; FANS]) -> Self {
        let on_steps = duty.map(duty_to_steps);
        let mut start = 0;
        let mut offs = [(0u16, 0u32); FANS];
        let mut len = 0;

        for fan in 0..FANS {
            let steps = on_steps[fan];
            if steps == 0 {
                start |= reset_bit(fan);
                continue;
            }
            start |= set_bit(fan);
            if steps >= cfg::PWM_STEPS {
                continue; // 100%: never turns off.
            }
            // Merge with an existing edge at the same step if there is one.
            match offs[..len].iter_mut().find(|(s, _)| *s as usize == steps) {
                Some((_, mask)) => *mask |= reset_bit(fan),
                None => {
                    offs[len] = (steps as u16, reset_bit(fan));
                    len += 1;
                }
            }
        }

        offs[..len].sort_unstable_by_key(|(step, _)| *step);
        Self { start, offs, len }
    }

    pub fn offs(&self) -> &[(u16, u32)] {
        &self.offs[..self.len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled() -> Rcs {
        let mut rcs = Rcs::new();
        rcs.enable();
        rcs
    }

    #[test]
    fn starts_disabled_and_rejects_commands() {
        let mut rcs = Rcs::new();
        assert_eq!(rcs.state(), RcsState::Disabled);
        assert_eq!(rcs.set_duty([0.5; 4], 1.0), Err(RcsError::NotEnabled));
        assert_eq!(rcs.latch(), [0.0; 4]);
    }

    #[test]
    fn invalid_duty_keeps_previous() {
        let mut rcs = enabled();
        rcs.set_duty([0.2; 4], 1.0).unwrap();
        assert_eq!(
            rcs.set_duty([f32::NAN, 0., 0., 0.], 1.0),
            Err(RcsError::Invalid)
        );
        assert_eq!(rcs.set_duty([1.5, 0., 0., 0.], 1.0), Err(RcsError::Invalid));
        assert_eq!(rcs.latch(), [0.2; 4]);
    }

    #[test]
    fn increases_are_slew_limited_decreases_are_not() {
        let mut rcs = enabled();
        // 5 /s * 0.01 s = 0.05 per call.
        let out = rcs.set_duty([1.0, 0.0, 1.0, 0.0], 0.01).unwrap();
        assert!((out[0] - 0.05).abs() < 1e-6);
        for _ in 0..30 {
            rcs.set_duty([1.0, 0.0, 1.0, 0.0], 0.01).unwrap();
        }
        let out = rcs.set_duty([0.0; 4], 0.01).unwrap();
        assert_eq!(out, [0.0; 4]);
    }

    #[test]
    fn duty_only_reaches_pins_at_latch() {
        let mut rcs = enabled();
        rcs.set_duty([0.04, 0.0, 0.0, 0.0], 1.0).unwrap();
        assert_eq!(rcs.active(), [0.0; 4]);
        assert_eq!(rcs.latch()[0], 0.04);
    }

    #[test]
    fn safe_is_immediate_and_sticky() {
        let mut rcs = enabled();
        rcs.set_duty([0.5; 4], 1.0).unwrap();
        rcs.latch();
        rcs.safe();
        assert_eq!(rcs.active(), [0.0; 4]);
        rcs.enable(); // Must not leave safed by itself.
        assert_eq!(rcs.state(), RcsState::Safed);
        assert_eq!(rcs.set_duty([0.5; 4], 1.0), Err(RcsError::NotEnabled));
        rcs.clear_safe();
        assert_eq!(rcs.state(), RcsState::Disabled);
    }

    #[test]
    fn overcurrent_needs_debounce_then_latches_one_fan() {
        let mut rcs = enabled();
        rcs.set_duty([0.5; 4], 1.0).unwrap();
        rcs.latch();
        let high = [0.0, 0.9, 0.0, 0.0];
        assert_eq!(rcs.update_current(high, 30), 0);
        assert_eq!(rcs.update_current(high, 30), 0b0010);
        assert_eq!(rcs.active(), [0.5, 0.0, 0.5, 0.5]);
        // Stays off even when commanded and current drops.
        rcs.update_current([0.0; 4], 10);
        assert_eq!(rcs.set_duty([0.5; 4], 1.0).unwrap()[1], 0.0);
    }

    #[test]
    fn brief_surge_does_not_trip() {
        let mut rcs = enabled();
        rcs.update_current([0.9, 0.0, 0.0, 0.0], 30);
        rcs.update_current([0.1, 0.0, 0.0, 0.0], 10);
        rcs.update_current([0.9, 0.0, 0.0, 0.0], 30);
        assert_eq!(rcs.faulted(), 0);
    }

    #[test]
    fn nan_current_counts_as_over() {
        let mut rcs = enabled();
        rcs.update_current([f32::NAN, 0., 0., 0.], 60);
        assert_eq!(rcs.faulted(), 1);
    }

    #[test]
    fn saturation_is_reported() {
        let mut rcs = enabled();
        rcs.update_current([0.65, 0.1, 0.0, 0.0], 1);
        assert_eq!(rcs.report().current_saturated, 0b0001);
    }

    #[test]
    fn steps_are_exact_at_the_ends() {
        assert_eq!(duty_to_steps(0.0), 0);
        assert_eq!(duty_to_steps(-1.0), 0);
        assert_eq!(duty_to_steps(f32::NAN), 0);
        assert_eq!(duty_to_steps(1.0), cfg::PWM_STEPS);
        assert_eq!(duty_to_steps(0.004), 0);
        assert_eq!(duty_to_steps(0.006), 1);
    }

    #[test]
    fn safe_word_clears_pd0_to_pd3() {
        assert_eq!(SAFE_BSRR, 0x000F_0000);
    }

    #[test]
    fn bsrr_table_matches_duty() {
        let mut table = [0u32; cfg::PWM_STEPS];
        fill_bsrr_table(&[0.6, 0.0, 1.0, 0.25], &mut table);

        let on_count = |fan: usize| table.iter().filter(|w| **w & set_bit(fan) != 0).count();
        assert_eq!(on_count(0), 60);
        assert_eq!(on_count(1), 0);
        assert_eq!(on_count(2), 100);
        assert_eq!(on_count(3), 25);

        // Every word states every fan exactly once.
        for word in table {
            for fan in 0..FANS {
                let set = word & set_bit(fan) != 0;
                let reset = word & reset_bit(fan) != 0;
                assert!(set ^ reset);
            }
        }
    }

    #[test]
    fn edge_schedule_sorts_and_merges() {
        let sched = EdgeSchedule::new(&[0.6, 0.0, 0.3, 0.6]);
        assert_eq!(
            sched.start,
            set_bit(0) | reset_bit(1) | set_bit(2) | set_bit(3)
        );
        assert_eq!(
            sched.offs(),
            &[(30, reset_bit(2)), (60, reset_bit(0) | reset_bit(3))]
        );
    }

    #[test]
    fn edge_schedule_full_and_zero_have_no_edges() {
        let sched = EdgeSchedule::new(&[1.0, 0.0, 1.0, 0.0]);
        assert!(sched.offs().is_empty());
    }
}
