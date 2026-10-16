//! The EC's boss: checks every message from the FC, notices when the FC goes
//! quiet, and decides what each output is allowed to do.
//!
//! # Rules
//! - A demand only counts if it decodes, has the right protocol version,
//!   has finite in-range values, and has a newer sequence number. Anything
//!   else is dropped and does **not** reset the link timer.
//! - If outputs are live and no valid demand arrives for
//!   `DEMAND_TIMEOUT_MS`, the EC safes itself. It does not wait for the FC.
//! - `RequestedMode::Safe` from the FC safes immediately.
//! - Leaving the safed state takes a deliberate `RequestedMode::Disarm` from
//!   the FC (an acknowledgement). Simply resuming `Arm` messages does not.
//!   After that, arming starts over, including the ESC's arming hold.
//! - Actuator problems (link lost, overload, voltage) are reported to the FC
//!   but do not safe the EC by themselves: losing steering is a
//!   vehicle-level decision (abort or not) that belongs to the FC.

use crate::{
    config::{self, link::DEMAND_TIMEOUT_MS},
    pa12::{error_bit, Pa12Link},
    propeller::{Esc, EscInputs, EscOutput, EscState},
    protocol::{fault, DemandError, EcDemand, EcState, EcStatus, RailReport, RequestedMode},
    rcs::Rcs,
};

/// Readings the hardware layer collects and hands over for the status
/// message. Values are already in volts, amps and degrees C.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Measurements {
    /// `[22.2 V, 12 V, 4.4 V, 3.3 V]`.
    pub rails: [RailReport; 4],
    pub motor_rail_v: f32,
    pub board_temp_c: [f32; 2],
}

/// What the hardware layer must do after a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickOutput {
    /// Put this on the ESC timer.
    pub esc: EscOutput,
    /// The EC safed during this tick: write `rcs::SAFE_BSRR` to GPIOD now
    /// instead of waiting for the next flicker cycle.
    pub safe_now: bool,
}

pub struct Supervisor {
    boot_ms: u64,
    last_tick_ms: u64,
    last_seq: Option<u32>,
    last_valid_ms: Option<u64>,
    demand: Option<EcDemand>,
    safed: bool,
    faults: u32,
    pub rcs: Rcs,
    pub esc: Esc,
    /// `[A1, A2]`.
    pub tvc: [Pa12Link; 2],
}

impl Supervisor {
    /// Call once at power-up.
    pub fn new(now_ms: u64) -> Self {
        Self {
            boot_ms: now_ms,
            last_tick_ms: now_ms,
            last_seq: None,
            last_valid_ms: None,
            demand: None,
            safed: false,
            faults: 0,
            rcs: Rcs::new(),
            esc: Esc::new(now_ms),
            tvc: [
                Pa12Link::new(config::pa12::ID),
                Pa12Link::new(config::pa12::ID),
            ],
        }
    }

    fn link_fresh(&self, now_ms: u64) -> bool {
        self.last_valid_ms
            .is_some_and(|t| now_ms.saturating_sub(t) <= DEMAND_TIMEOUT_MS)
    }

    /// Handles one UDP payload from the FC.
    pub fn on_demand_bytes(&mut self, bytes: &[u8], now_ms: u64) -> Result<(), DemandError> {
        self.on_demand(EcDemand::decode(bytes)?, now_ms)
    }

    pub fn on_demand(&mut self, demand: EcDemand, now_ms: u64) -> Result<(), DemandError> {
        let demand = *demand.validate()?.get();

        // Sequence numbers must increase. Exception: once the link has gone
        // stale, any number is accepted, so a rebooted FC starting again
        // from 0 is not locked out.
        if let Some(last) = self.last_seq {
            if demand.seq <= last && self.link_fresh(now_ms) {
                return Err(DemandError::StaleSequence);
            }
        }

        self.last_seq = Some(demand.seq);
        self.last_valid_ms = Some(now_ms);
        self.demand = Some(demand);

        match demand.mode {
            RequestedMode::Safe => self.enter_safe(fault::FC_COMMANDED_SAFE),
            RequestedMode::Disarm if self.safed => self.clear_safe(),
            _ => {}
        }
        Ok(())
    }

    fn enter_safe(&mut self, reason: u32) {
        self.faults |= reason;
        if self.safed {
            return;
        }
        self.safed = true;
        self.rcs.safe();
        self.esc.safe();
        for actuator in &mut self.tvc {
            actuator.safe();
        }
    }

    fn clear_safe(&mut self) {
        self.safed = false;
        self.faults = 0;
        self.rcs.clear_safe();
        self.esc.clear_safe();
    }

    /// Whether anything could currently be moving.
    fn outputs_live(&self) -> bool {
        self.rcs.state() == crate::rcs::RcsState::Enabled
            || matches!(self.esc.state(), EscState::Arming { .. } | EscState::Armed)
    }

    /// Runs the control logic. Call at a steady rate (e.g. 1 kHz or the FC
    /// rate) from the main loop.
    pub fn tick(&mut self, now_ms: u64) -> TickOutput {
        let dt_s = now_ms.saturating_sub(self.last_tick_ms) as f32 / 1_000.0;
        self.last_tick_ms = now_ms;
        let was_safed = self.safed;
        let fresh = self.link_fresh(now_ms);

        if !fresh && self.outputs_live() {
            self.enter_safe(fault::LINK_TIMEOUT);
        }

        let arm = !self.safed && fresh && self.demand.is_some_and(|d| d.mode == RequestedMode::Arm);

        let throttle = match self.demand {
            Some(d) if arm => d.main_throttle,
            _ => 0.0,
        };

        let esc = self.esc.tick(
            now_ms,
            dt_s,
            EscInputs {
                arm_requested: arm,
                link_fresh: fresh,
                interlocks_ok: !self.safed,
                throttle,
            },
        );

        // Fans only run once the whole EC is armed (ESC through its arming
        // hold), so "Armed" means the same thing for every output.
        if arm && self.esc.state() == EscState::Armed {
            self.rcs.enable();
            if let Some(d) = self.demand {
                // Already validated, so this cannot fail.
                let _ = self.rcs.set_duty(d.rcs_duty, dt_s);
            }
        } else if !self.safed {
            self.rcs.disable();
        }

        // Actuators follow the FC whenever it is talking and the EC is not
        // safed, even when disarmed, so the gimbal can be checked on the pad.
        if fresh && !self.safed {
            if let Some(d) = self.demand {
                for (actuator, mm) in self.tvc.iter_mut().zip(d.tvc_extension_mm) {
                    actuator.set_goal_mm(mm);
                }
            }
        }

        self.update_tvc_faults();

        TickOutput {
            esc,
            safe_now: self.safed && !was_safed,
        }
    }

    /// Feeds fan currents (amps, averaged per flicker cycle).
    pub fn on_fan_currents(&mut self, amps: [f32; 4], dt_ms: u64) {
        if self.rcs.update_current(amps, dt_ms) != 0 {
            self.faults |= fault::RCS_OVERCURRENT;
        }
    }

    fn update_tvc_faults(&mut self) {
        let mut flags = 0;
        for actuator in &self.tvc {
            if !actuator.link_ok() {
                flags |= fault::TVC_COMM_LOST;
            }
            if actuator.error_bits() & error_bit::OVERLOAD != 0 {
                flags |= fault::TVC_OVERLOAD;
            }
            if actuator.error_bits() & error_bit::INPUT_VOLTAGE != 0 {
                flags |= fault::TVC_VOLTAGE;
            }
        }
        let tvc_mask = fault::TVC_COMM_LOST | fault::TVC_OVERLOAD | fault::TVC_VOLTAGE;
        self.faults = (self.faults & !tvc_mask) | flags;
    }

    pub fn state(&self) -> EcState {
        if self.safed {
            return EcState::Safed;
        }
        match self.esc.state() {
            EscState::BootSafe { .. } => EcState::BootSafe,
            EscState::Arming { .. } => EcState::Arming,
            EscState::Armed => EcState::Armed,
            EscState::Disarmed | EscState::Safed => EcState::Disarmed,
        }
    }

    pub fn faults(&self) -> u32 {
        self.faults
    }

    pub fn status(&self, now_ms: u64, m: &Measurements) -> EcStatus {
        EcStatus {
            last_seq: self.last_seq.unwrap_or(0),
            uptime_ms: now_ms.saturating_sub(self.boot_ms) as u32,
            state: self.state(),
            faults: self.faults,
            rcs: self.rcs.report(),
            esc: self.esc.report(),
            tvc: [self.tvc[0].report(), self.tvc[1].report()],
            rails: m.rails,
            motor_rail_v: m.motor_rail_v,
            board_temp_c: m.board_temp_c,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::esc::{ARM_HOLD_MS, BOOT_MIN_HOLD_MS, MIN_PULSE_US};

    fn demand(seq: u32, mode: RequestedMode) -> EcDemand {
        EcDemand {
            seq,
            mode,
            tvc_extension_mm: [10.0, 12.0],
            rcs_duty: [0.6, 0.0, 0.6, 0.0],
            main_throttle: 0.0,
        }
    }

    /// Simulates the FC sending a demand and the EC ticking every 10 ms.
    struct Sim {
        sup: Supervisor,
        now: u64,
        seq: u32,
    }

    impl Sim {
        fn new() -> Self {
            Self {
                sup: Supervisor::new(0),
                now: 0,
                seq: 0,
            }
        }

        fn step(&mut self, mode: RequestedMode, throttle: f32) -> TickOutput {
            self.now += 10;
            self.seq += 1;
            let d = EcDemand {
                main_throttle: throttle,
                ..demand(self.seq, mode)
            };
            // Through the real byte path, like a UDP packet from the FC.
            self.sup.on_demand_bytes(&d.encode(), self.now).unwrap();
            self.sup.tick(self.now)
        }

        fn run(&mut self, ms: u64, mode: RequestedMode, throttle: f32) {
            for _ in 0..ms / 10 {
                self.step(mode, throttle);
            }
        }

        fn silent(&mut self, ms: u64) -> Option<TickOutput> {
            let mut out = None;
            for _ in 0..ms / 10 {
                self.now += 10;
                out = Some(self.sup.tick(self.now));
            }
            out
        }

        fn armed() -> Self {
            let mut sim = Self::new();
            sim.run(BOOT_MIN_HOLD_MS + ARM_HOLD_MS + 50, RequestedMode::Arm, 0.0);
            assert_eq!(sim.sup.state(), EcState::Armed);
            sim
        }
    }

    #[test]
    fn rejects_old_and_duplicate_sequence() {
        let mut sup = Supervisor::new(0);
        sup.on_demand(demand(5, RequestedMode::Disarm), 0).unwrap();
        assert_eq!(
            sup.on_demand(demand(5, RequestedMode::Arm), 1),
            Err(DemandError::StaleSequence)
        );
        assert_eq!(
            sup.on_demand(demand(4, RequestedMode::Arm), 2),
            Err(DemandError::StaleSequence)
        );
    }

    #[test]
    fn rebooted_fc_accepted_after_timeout() {
        let mut sup = Supervisor::new(0);
        sup.on_demand(demand(500, RequestedMode::Disarm), 0)
            .unwrap();
        let later = DEMAND_TIMEOUT_MS + 1;
        assert!(sup
            .on_demand(demand(1, RequestedMode::Disarm), later)
            .is_ok());
    }

    #[test]
    fn bad_bytes_do_not_count_as_life() {
        let mut sim = Sim::armed();
        for _ in 0..20 {
            sim.now += 10;
            let _ = sim.sup.on_demand_bytes(&[1, 2, 3], sim.now);
            sim.sup.tick(sim.now);
        }
        assert_eq!(sim.sup.state(), EcState::Safed);
        assert_ne!(sim.sup.faults() & fault::LINK_TIMEOUT, 0);
    }

    #[test]
    fn fans_follow_only_when_armed() {
        let mut sim = Sim::new();
        sim.run(500, RequestedMode::Disarm, 0.0);
        assert_eq!(sim.sup.rcs.latch(), [0.0; 4]);

        let mut sim = Sim::armed();
        sim.run(500, RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.rcs.latch(), [0.6, 0.0, 0.6, 0.0]);
    }

    #[test]
    fn link_loss_safes_everything_once() {
        let mut sim = Sim::armed();
        sim.run(500, RequestedMode::Arm, 0.4);
        sim.sup.rcs.latch();

        // Within the timeout: still armed.
        sim.silent(DEMAND_TIMEOUT_MS);
        assert_eq!(sim.sup.state(), EcState::Armed);

        let out = sim.silent(10).unwrap();
        assert!(out.safe_now);
        assert_eq!(sim.sup.state(), EcState::Safed);
        assert_eq!(sim.sup.rcs.active(), [0.0; 4]);
        assert_eq!(out.esc.pulse_us, Some(MIN_PULSE_US));

        // safe_now is a one-shot.
        assert!(!sim.silent(10).unwrap().safe_now);
    }

    #[test]
    fn no_timeout_fault_while_idle_on_the_pad() {
        let mut sim = Sim::new();
        sim.silent(2_000);
        assert_eq!(sim.sup.faults() & fault::LINK_TIMEOUT, 0);
        assert_ne!(sim.sup.state(), EcState::Safed);
    }

    #[test]
    fn resuming_arm_does_not_unsafe_but_disarm_ack_does() {
        let mut sim = Sim::armed();
        sim.step(RequestedMode::Safe, 0.0);
        assert_eq!(sim.sup.state(), EcState::Safed);

        sim.run(2_000, RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Safed);
        assert_eq!(sim.sup.rcs.latch(), [0.0; 4]);

        sim.step(RequestedMode::Disarm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Disarmed);
        // Only the actuator-link flag remains: nothing answers in this test.
        assert_eq!(sim.sup.faults(), fault::TVC_COMM_LOST);

        // Re-arming goes through the arming hold again.
        sim.step(RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Arming);
        sim.run(ARM_HOLD_MS + 20, RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Armed);
    }

    #[test]
    fn actuators_get_goals_when_disarmed_but_not_when_safed() {
        let mut sim = Sim::new();
        sim.step(RequestedMode::Disarm, 0.0);
        assert_eq!(sim.sup.tvc[0].report().requested_mm, 10.0);

        let mut sim = Sim::armed();
        sim.step(RequestedMode::Safe, 0.0);
        let d = EcDemand {
            tvc_extension_mm: [20.0, 20.0],
            ..demand(sim.seq + 1, RequestedMode::Arm)
        };
        sim.now += 10;
        sim.sup.on_demand(d, sim.now).unwrap();
        sim.sup.tick(sim.now);
        assert_eq!(sim.sup.tvc[0].report().requested_mm, 10.0);
    }

    #[test]
    fn fans_stay_off_until_esc_finishes_arming() {
        let mut sim = Sim::new();
        sim.run(BOOT_MIN_HOLD_MS + 500, RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Arming);
        assert_eq!(sim.sup.rcs.latch(), [0.0; 4]);
    }

    #[test]
    fn fan_overcurrent_is_flagged_but_not_a_full_safe() {
        let mut sim = Sim::armed();
        sim.sup.on_fan_currents([0.9, 0.0, 0.0, 0.0], 100);
        assert_ne!(sim.sup.faults() & fault::RCS_OVERCURRENT, 0);
        sim.run(100, RequestedMode::Arm, 0.0);
        assert_eq!(sim.sup.state(), EcState::Armed);
        assert_eq!(sim.sup.rcs.latch()[0], 0.0);
    }

    #[test]
    fn status_reports_state_and_round_trips() {
        let sim = Sim::armed();
        let status = sim.sup.status(sim.now, &Measurements::default());
        assert_eq!(status.state, EcState::Armed);
        assert_eq!(status.last_seq, sim.seq);
        let bytes = status.encode();
        assert_eq!(bytes.len(), crate::protocol::STATUS_LEN);
        assert_eq!(&bytes[..2], &crate::protocol::MAGIC.to_le_bytes());
    }
}
