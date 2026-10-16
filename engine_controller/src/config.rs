//! Every number the engine controller depends on, in one place.
//!
//! Tags used below:
//! - `SCHEMATIC`: read off the CTV Engine Controller schematic (rev 1.0).
//! - `DATASHEET`: from a part's datasheet or manual.
//! - `ASSUMED`: a placeholder guess. Replace it once the real part is known or
//!   a bench test has measured it.

/// FC <-> EC link.
pub mod link {
    /// If no valid demand arrives for this long, the EC safes its outputs.
    /// ASSUMED: about 10 missed packets at a 100 Hz FC loop.
    pub const DEMAND_TIMEOUT_MS: u64 = 100;
}

/// RCS roll fans (M1-M4).
pub mod rcs {
    /// Number of fans.
    pub const FAN_COUNT: usize = 4;

    /// GPIOD pin number driving each fan's switch, indexed by fan (M1..M4).
    /// SCHEMATIC: M1-EN = PD3, M2-EN = PD2, M3-EN = PD1, M4-EN = PD0.
    pub const GPIOD_PIN: [u8; FAN_COUNT] = [3, 2, 1, 0];

    /// How many times per second each fan is flickered on/off.
    /// ASSUMED: pick from the fan's datasheet once the fan is known.
    pub const PWM_FREQ_HZ: u32 = 1_000;

    /// Steps per flicker cycle, i.e. duty resolution (100 = 1% steps).
    /// ASSUMED.
    pub const PWM_STEPS: usize = 100;

    /// Fastest a fan's duty may change, in full-scale per second.
    /// 5.0 means 0% -> 100% takes at least 0.2 s. Keeps the shared 4.4 V
    /// supply (which also feeds the MCU) from sagging on a sudden step.
    /// ASSUMED.
    pub const MAX_DUTY_SLEW_PER_S: f32 = 5.0;

    /// Fan current sense gain in volts per amp.
    /// SCHEMATIC: 50 mOhm shunt x INA180A3 (gain 100) = 5 V/A.
    pub const SENSE_V_PER_A: f32 = 5.0;

    /// ADC full-scale voltage. SCHEMATIC: VREF+ is tied to the 3.3 V rail.
    pub const ADC_FULL_SCALE_V: f32 = 3.3;

    /// Readings at or above this are pegged at the top of the sensor's range
    /// and only mean "at least this much".
    /// Derived: 3.3 V / 5 V/A = 0.66 A, minus a little margin.
    pub const SENSE_SATURATION_A: f32 = 0.64;

    /// Current above which a fan counts as overloaded or stalled.
    /// ASSUMED: set from the fan's datasheet. It has to sit below the sensor
    /// saturation point to be detectable at all.
    pub const OVERCURRENT_A: f32 = 0.60;

    /// How long the current must stay over the limit before the fan is shut
    /// off. Keeps a normal spin-up surge from tripping it. ASSUMED.
    pub const OVERCURRENT_DEBOUNCE_MS: u64 = 50;
}

/// Main propeller ESCs: 2x ZTW Mantis G2 35A, one shared signal on PA5.
/// The Mantis is rated 2-4S LiPo; the schematic feeds the ESCs 22.2 V (6S).
/// Resolve that before powering them.
pub mod esc {
    /// Pulse width meaning "zero throttle". ASSUMED: standard RC value. The
    /// Mantis learns its endpoints by throttle calibration, so calibrate it
    /// with exactly these two values.
    pub const MIN_PULSE_US: u16 = 1_000;

    /// Pulse width meaning "full throttle". ASSUMED.
    pub const MAX_PULSE_US: u16 = 2_000;

    /// Pulses per second sent to the ESC. ASSUMED: 50 Hz is accepted by
    /// nearly every RC ESC; many accept 400 Hz.
    pub const FRAME_HZ: u32 = 50;

    /// After power-up the ESC is fed minimum throttle for this long before
    /// the EC will consider arming. Many ESCs need to see minimum throttle at
    /// power-up, otherwise they enter calibration/programming mode. ASSUMED.
    pub const BOOT_MIN_HOLD_MS: u64 = 3_000;

    /// Arming holds minimum throttle this long (with every condition still
    /// true) before throttle is allowed through. ASSUMED.
    pub const ARM_HOLD_MS: u64 = 1_000;

    /// Fastest throttle may change, full-scale per second. ASSUMED.
    pub const MAX_THROTTLE_SLEW_PER_S: f32 = 2.0;

    /// Highest throttle the EC will ever pass through (0..=1).
    /// ASSUMED: raise once the thrust stand says it is safe.
    pub const MAX_THROTTLE: f32 = 1.0;

    /// What the ESC line does once the EC has safed.
    /// ASSUMED: keep sending minimum-throttle pulses. Whether "stop pulsing"
    /// is safer depends on how the ESC reacts to signal loss; bench test both.
    pub const SAFED_OUTPUT: crate::propeller::SafedOutput = crate::propeller::SafedOutput::MinPulse;
}

/// PA-12-T TVC actuators (A1 on UART7, A2 on UART5).
pub mod pa12 {
    /// DATASHEET: default baud rate.
    pub const BAUD: u32 = 57_600;

    /// Actuator ID. Each actuator sits on its own UART, so both can keep
    /// the factory ID. DATASHEET: factory default ID is 0.
    pub const ID: u8 = 0;

    /// DATASHEET: the 1.06" model's factory long-stroke limit is 3686 counts
    /// = 1.06 in = 26.924 mm.
    pub const MAX_COUNTS: u16 = 3_686;

    /// Counts per millimetre of rod travel. Derived from the line above.
    /// Calibrate on the bench with calipers.
    pub const COUNTS_PER_MM: f32 = 3_686.0 / 26.924;

    /// Usable stroke in mm, matching `MAX_COUNTS`.
    pub const STROKE_MM: f32 = 26.924;

    /// DATASHEET: recommended wait after a write before the next transfer.
    pub const WRITE_GAP_MS: u64 = 5;

    /// DATASHEET: recommended wait after a read before the next transfer.
    pub const READ_GAP_MS: u64 = 10;

    /// Give up on a reply after this long. ASSUMED.
    pub const REPLY_TIMEOUT_MS: u64 = 20;

    /// Missed replies in a row before the actuator link counts as lost.
    /// ASSUMED.
    pub const MAX_MISSED_REPLIES: u8 = 5;

    /// Ask for the actuator's position once every N goal writes. ASSUMED.
    pub const READ_EVERY_N_WRITES: u8 = 2;

    /// What the actuators do when the EC safes.
    /// ASSUMED: hold the last commanded position (they stay powered from
    /// 12 V regardless). Centering is the other option; team decision.
    pub const SAFED_ACTION: crate::pa12::SafedAction = crate::pa12::SafedAction::HoldLast;
}
