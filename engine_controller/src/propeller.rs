//! Main propeller ESCs.
//!
//! # How the hardware works
//! PA5 (ESC-CTRL) goes to J403 pins 1 and 2, so **both** ESCs on the
//! contra-rotating pair hear the same signal: same throttle, same arming,
//! same endpoint calibration. There is no RPM or per-ESC current feedback.
//!
//! The signal is a repeating pulse whose width means throttle. The standard
//! RC convention (ASSUMED until the ESC manual is checked) is ~1000 us = zero
//! and ~2000 us = full, repeated `FRAME_HZ` times a second. PA5 can be
//! TIM2_CH1, so the timer hardware makes the pulse with no CPU work.
//!
//! # Reset and signal loss
//! - MCU reset or crash: PA5 floats (H7 pins reset to analog / high-Z) and
//!   the board has no pull-down on that line, so the ESC sees *no* signal.
//!   What it does then depends on its firmware: bench test it.
//! - EC safed: the line does `config::esc::SAFED_OUTPUT`, either keep
//!   sending minimum-throttle pulses or stop pulsing. These are different:
//!   "no pulses" makes the ESC run its own signal-loss behavior, while "min
//!   pulse" tells it to idle. Bench test both before choosing.
//!
//! # Endpoint calibration
//! Calibration teaches an ESC what "zero" and "full" pulses look like. Both
//! ESCs share the wire, so they are calibrated together through it, or each
//! is plugged in alone during setup. This firmware never runs calibration:
//! it only produces pulses inside the configured range, and it holds minimum
//! throttle for `BOOT_MIN_HOLD_MS` after power-up so an ESC never powers up
//! seeing high throttle (which on many ESCs starts calibration mode).

use crate::{config::esc as cfg, protocol::EscReport};

/// What the ESC line does once the EC has safed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SafedOutput {
    /// Keep pulsing at minimum throttle.
    MinPulse,
    /// Stop pulsing; the line sits low.
    NoPulse,
}

/// ESC state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EscState {
    /// Just powered up: minimum throttle until `BOOT_MIN_HOLD_MS` passes.
    BootSafe { since_ms: u64 },
    /// Minimum throttle, waiting for an arm request.
    Disarmed,
    /// Minimum throttle while every arming condition must stay true for
    /// `ARM_HOLD_MS`.
    Arming { since_ms: u64 },
    /// Throttle follows the FC.
    Armed,
    /// Fault: output per `SAFED_OUTPUT` until [`Esc::clear_safe`].
    Safed,
}

/// What the supervisor feeds the ESC logic each tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscInputs {
    /// The FC wants outputs live.
    pub arm_requested: bool,
    /// A valid demand arrived within the link timeout.
    pub link_fresh: bool,
    /// Other conditions that must hold to arm (e.g. no board faults). The
    /// supervisor decides what goes in here.
    pub interlocks_ok: bool,
    /// Requested throttle `0..=1`.
    pub throttle: f32,
}

/// What to put on the wire this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EscOutput {
    /// Pulse width, or `None` to stop pulsing.
    pub pulse_us: Option<u16>,
}

/// ESC arming, throttle limits and safing.
#[derive(Clone, Debug)]
pub struct Esc {
    state: EscState,
    requested: f32,
    applied: f32,
    output: EscOutput,
}

impl Esc {
    /// Call once at power-up.
    pub fn new(now_ms: u64) -> Self {
        Self {
            state: EscState::BootSafe { since_ms: now_ms },
            requested: 0.0,
            applied: 0.0,
            output: EscOutput {
                pulse_us: Some(cfg::MIN_PULSE_US),
            },
        }
    }

    pub fn state(&self) -> EscState {
        self.state
    }

    /// Forces the safed state immediately.
    pub fn safe(&mut self) {
        self.state = EscState::Safed;
        self.applied = 0.0;
        self.output = safed_output();
    }

    /// Acknowledges a fault: safed -> disarmed.
    pub fn clear_safe(&mut self) {
        if self.state == EscState::Safed {
            self.state = EscState::Disarmed;
        }
    }

    /// Advances the state machine. Call at a steady rate (e.g. the
    /// supervisor's tick), `dt_s` seconds after the previous call.
    pub fn tick(&mut self, now_ms: u64, dt_s: f32, inp: EscInputs) -> EscOutput {
        // NaN or out-of-range throttle is treated as zero.
        let throttle = if inp.throttle.is_finite() {
            inp.throttle.clamp(0.0, cfg::MAX_THROTTLE)
        } else {
            0.0
        };
        self.requested = throttle;

        let may_arm = inp.arm_requested && inp.link_fresh && inp.interlocks_ok;

        self.state = match self.state {
            EscState::BootSafe { since_ms } => {
                if now_ms.saturating_sub(since_ms) >= cfg::BOOT_MIN_HOLD_MS {
                    EscState::Disarmed
                } else {
                    self.state
                }
            }
            EscState::Disarmed => {
                if may_arm && throttle == 0.0 {
                    EscState::Arming { since_ms: now_ms }
                } else {
                    EscState::Disarmed
                }
            }
            EscState::Arming { since_ms } => {
                if !(may_arm && throttle == 0.0) {
                    EscState::Disarmed
                } else if now_ms.saturating_sub(since_ms) >= cfg::ARM_HOLD_MS {
                    EscState::Armed
                } else {
                    self.state
                }
            }
            EscState::Armed => {
                if !inp.link_fresh {
                    EscState::Safed
                } else if !(inp.arm_requested && inp.interlocks_ok) {
                    EscState::Disarmed
                } else {
                    EscState::Armed
                }
            }
            EscState::Safed => EscState::Safed,
        };

        self.applied = match self.state {
            EscState::Armed => {
                let max_up = cfg::MAX_THROTTLE_SLEW_PER_S * dt_s.max(0.0);
                if throttle > self.applied {
                    throttle.min(self.applied + max_up)
                } else {
                    throttle
                }
            }
            _ => 0.0,
        };

        self.output = match self.state {
            EscState::Safed => safed_output(),
            _ => EscOutput {
                pulse_us: Some(throttle_to_pulse_us(self.applied)),
            },
        };
        self.output
    }

    pub fn report(&self) -> EscReport {
        EscReport {
            requested_throttle: self.requested,
            applied_throttle: self.applied,
            pulse_us: self.output.pulse_us.unwrap_or(0),
        }
    }
}

fn safed_output() -> EscOutput {
    EscOutput {
        pulse_us: match cfg::SAFED_OUTPUT {
            SafedOutput::MinPulse => Some(cfg::MIN_PULSE_US),
            SafedOutput::NoPulse => None,
        },
    }
}

/// Maps throttle `0..=1` linearly onto the calibrated pulse range.
pub fn throttle_to_pulse_us(throttle: f32) -> u16 {
    let t = if throttle.is_finite() {
        throttle.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let span = (cfg::MAX_PULSE_US - cfg::MIN_PULSE_US) as f32;
    cfg::MIN_PULSE_US + (t * span + 0.5) as u16
}

/// Register values for the timer making the ESC pulse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PwmTimerConfig {
    /// Goes in TIMx_PSC. Chosen so the timer counts once per microsecond.
    pub prescaler: u16,
    /// Goes in TIMx_ARR. One pulse every `period + 1` microseconds.
    pub period: u32,
}

impl PwmTimerConfig {
    /// Value for TIMx_CCRx: the pulse width in microseconds, or 0 for no
    /// pulse (line stays low).
    pub fn compare(&self, out: EscOutput) -> u32 {
        out.pulse_us.map_or(0, u32::from)
    }
}

/// Works out timer settings for a 1 us tick from the timer's input clock.
///
/// `timer_clock_hz` must come from the real clock tree. This board has no
/// external crystal, so the MCU runs from its internal HSI oscillator
/// through the PLL; read the resulting TIM2 kernel clock from the clock
/// setup code. Returns `None` if the clock is not a whole number of MHz or
/// the frame is too short to hold the longest pulse.
pub fn pwm_timer_config(timer_clock_hz: u32, frame_hz: u32) -> Option<PwmTimerConfig> {
    if frame_hz == 0 || !timer_clock_hz.is_multiple_of(1_000_000) {
        return None;
    }
    let prescaler = u16::try_from(timer_clock_hz / 1_000_000)
        .ok()?
        .checked_sub(1)?;
    let period = (1_000_000 / frame_hz).checked_sub(1)?;
    if period <= u32::from(cfg::MAX_PULSE_US) {
        return None;
    }
    Some(PwmTimerConfig { prescaler, period })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.01;

    fn go(throttle: f32) -> EscInputs {
        EscInputs {
            arm_requested: true,
            link_fresh: true,
            interlocks_ok: true,
            throttle,
        }
    }

    /// Runs `ms` milliseconds of 10 ms ticks.
    fn run(esc: &mut Esc, now: &mut u64, ms: u64, inp: EscInputs) -> EscOutput {
        let mut out = EscOutput { pulse_us: None };
        for _ in 0..ms / 10 {
            *now += 10;
            out = esc.tick(*now, DT, inp);
        }
        out
    }

    fn armed() -> (Esc, u64) {
        let mut esc = Esc::new(0);
        let mut now = 0;
        run(&mut esc, &mut now, cfg::BOOT_MIN_HOLD_MS, go(0.0));
        run(&mut esc, &mut now, cfg::ARM_HOLD_MS + 20, go(0.0));
        assert_eq!(esc.state(), EscState::Armed);
        (esc, now)
    }

    #[test]
    fn boot_holds_min_throttle_even_if_fc_wants_more() {
        let mut esc = Esc::new(0);
        let mut now = 0;
        let out = run(&mut esc, &mut now, 1_000, go(0.8));
        assert_eq!(out.pulse_us, Some(cfg::MIN_PULSE_US));
        assert!(matches!(esc.state(), EscState::BootSafe { .. }));
    }

    #[test]
    fn will_not_arm_with_throttle_up() {
        let mut esc = Esc::new(0);
        let mut now = 0;
        run(&mut esc, &mut now, cfg::BOOT_MIN_HOLD_MS + 2_000, go(0.3));
        assert_eq!(esc.state(), EscState::Disarmed);
    }

    #[test]
    fn arming_aborts_if_a_condition_drops() {
        let mut esc = Esc::new(0);
        let mut now = 0;
        run(&mut esc, &mut now, cfg::BOOT_MIN_HOLD_MS, go(0.0));
        run(&mut esc, &mut now, 500, go(0.0));
        assert!(matches!(esc.state(), EscState::Arming { .. }));
        let no_interlock = EscInputs {
            interlocks_ok: false,
            ..go(0.0)
        };
        run(&mut esc, &mut now, 10, no_interlock);
        assert_eq!(esc.state(), EscState::Disarmed);
    }

    #[test]
    fn armed_throttle_is_slew_limited_up_and_instant_down() {
        let (mut esc, mut now) = armed();
        // 2 /s * 0.01 s = 0.02 per tick.
        let out = run(&mut esc, &mut now, 10, go(1.0));
        assert_eq!(out.pulse_us, Some(1020));
        let out = run(&mut esc, &mut now, 10, go(0.0));
        assert_eq!(out.pulse_us, Some(cfg::MIN_PULSE_US));
    }

    #[test]
    fn lost_link_while_armed_safes() {
        let (mut esc, mut now) = armed();
        run(&mut esc, &mut now, 500, go(0.5));
        let lost = EscInputs {
            link_fresh: false,
            ..go(0.5)
        };
        let out = run(&mut esc, &mut now, 10, lost);
        assert_eq!(esc.state(), EscState::Safed);
        assert_eq!(out, safed_output());
    }

    #[test]
    fn safed_needs_explicit_clear_and_then_rearming() {
        let (mut esc, mut now) = armed();
        esc.safe();
        run(&mut esc, &mut now, 2_000, go(0.0));
        assert_eq!(esc.state(), EscState::Safed);
        esc.clear_safe();
        run(&mut esc, &mut now, 10, go(0.0));
        assert!(matches!(esc.state(), EscState::Arming { .. }));
    }

    #[test]
    fn disarm_request_drops_to_min() {
        let (mut esc, mut now) = armed();
        run(&mut esc, &mut now, 500, go(0.5));
        let disarm = EscInputs {
            arm_requested: false,
            ..go(0.5)
        };
        let out = run(&mut esc, &mut now, 10, disarm);
        assert_eq!(esc.state(), EscState::Disarmed);
        assert_eq!(out.pulse_us, Some(cfg::MIN_PULSE_US));
    }

    #[test]
    fn nan_throttle_is_zero() {
        let (mut esc, mut now) = armed();
        let out = run(&mut esc, &mut now, 10, go(f32::NAN));
        assert_eq!(out.pulse_us, Some(cfg::MIN_PULSE_US));
    }

    #[test]
    fn pulse_mapping_hits_endpoints() {
        assert_eq!(throttle_to_pulse_us(0.0), cfg::MIN_PULSE_US);
        assert_eq!(throttle_to_pulse_us(1.0), cfg::MAX_PULSE_US);
        assert_eq!(throttle_to_pulse_us(0.5), 1_500);
        assert_eq!(throttle_to_pulse_us(7.0), cfg::MAX_PULSE_US);
    }

    #[test]
    fn timer_config_for_200mhz_at_50hz() {
        let t = pwm_timer_config(200_000_000, 50).unwrap();
        assert_eq!(t.prescaler, 199);
        assert_eq!(t.period, 19_999);
        assert_eq!(
            t.compare(EscOutput {
                pulse_us: Some(1500)
            }),
            1500
        );
        assert_eq!(t.compare(EscOutput { pulse_us: None }), 0);
    }

    #[test]
    fn timer_config_rejects_bad_inputs() {
        assert_eq!(pwm_timer_config(200_500_000, 50), None);
        assert_eq!(pwm_timer_config(200_000_000, 0), None);
        // 1000 Hz frame is only 1000 us long, shorter than a 2000 us pulse.
        assert_eq!(pwm_timer_config(200_000_000, 1_000), None);
    }
}
