/// Fields published for each TF-Luna sample.
#[derive(Clone, Copy)]
pub enum Channel {
    /// Range in millimeters.
    Distance,
    /// Raw return signal strength.
    SignalStrength,
    /// Sensor chip temperature in degrees Celsius.
    Temperature,
}

impl Channel {
    /// Returns the key in `VehicleState.sensor_readings` for a channel.
    pub fn key(self, id: u8) -> String {
        let field = match self {
            Self::Distance => "distance",
            Self::SignalStrength => "signal_strength",
            Self::Temperature => "temperature",
        };
        format!("fc_lidar_{id}_{field}")
    }
}

/// Identifies the reserved namespace of FC-local LiDAR measurements.
pub fn is_lidar_measurement(key: &str) -> bool {
    key.starts_with("fc_lidar_")
}
