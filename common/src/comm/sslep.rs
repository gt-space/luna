use serde::{Deserialize, Serialize};
use postcard::experimental::max_size::MaxSize;

#[derive(Clone, Copy, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub enum SslepChannelType {
    BoardTemperature,
    Thermocouple,
    AdcVoltage,
}

#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct SslepSensorDataPoint {
    pub value: f64,
    pub timestamp: f64,
    pub channel: u32,
    pub channel_type: SslepChannelType,
}

//telemetry sample sent from sslep to flight computer
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub enum SslepDataPoint {
    Sensor(SslepSensorDataPoint),
    //no rbf
}

//not sure if i need sslepcontrolmessage
/// Commands sent from the servo to SSLEP.
//proposed control commands for the Pi to enable/disable parts of the circuitry

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum SslepControlMessage {
    /// Request precharge enable/disable.
    //SetPrecharge = charge the capacitors gradually before enabling main power connection
    //enables TLP241B relay
    SetPrecharge {
        enabled: bool,
    },

    /// Request mains enable/disable.
    // turns on electronically controlled switch IRL540N MOSFET
    SetMains {
        enabled: bool,
    },

    /// Request ESC enable/disable.
    //ESC receives PWM signal to send electric signals to power the motor
    //2 ESC channels A and B
    SetEscEnable {
        channel: u8,
        enabled: bool,
    },

    /// Request a PWM output setting.
    // duty cycle is the percentage of time a signal stays HIGH for one complete cycle (a period)
    SetPwm {
        channel: u8,
        duty_cycle: f32,
    },

    /// Request an abort.
    Abort,
}

