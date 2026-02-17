use serde::{Deserialize, Serialize};

/// EKF bias parameter structure
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct EkfBiasParameters {
    /// Quaternion representing vehicle attitude [w, x, y, z]
    pub quaternion: [f32; 4],
    /// Position [longitude, latitude, altitude] in degrees/meters
    pub lla_pos: [f32; 3],
    /// Accelerometer bias offset [x, y, z]
    pub a_bias: [f32; 3],
    /// Gyroscope bias offset [x, y, z]
    pub g_bias: [f32; 3],
    /// Acceleration scale factor [x, y, z]
    pub a_sf: [f32; 3],
    /// Gyro scale factor [x, y, z]
    pub g_sf: [f32; 3],
    /// Pressure offset for the altimeter pressure calculations
    pub alt_press_off: f32,
     /// Pressure offset for the filter pressure calculations
     pub filter_press_off: f32,
}