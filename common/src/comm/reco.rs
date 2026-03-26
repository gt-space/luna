use serde::{Deserialize, Serialize};

/// Represents a command intended for RECO from the sequence path.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub enum SequenceCommand {
  /// Informs all RECO MCUs that the rocket has launched.
  Launch,
  /// Requests that all RECO MCUs initialize (or reinitialize) their EKFs.
  InitEKF,
}

/// Represents a command intended for RECO configuration from the GUI path.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub enum GuiCommand {
  /// Sends process noise matrix parameters to RECO.
  ProcessNoiseMatrix(ProcessNoiseMatrix),
  /// Sends measurement noise matrix parameters to RECO.
  MeasurementNoiseMatrix(MeasurementNoiseMatrix),
  /// Sends an EKF state vector to RECO.
  EKFStateVector(EkfStateVector),
  /// Sends an initial covariance matrix to RECO.
  InitialCovarianceMatrix(InitialCovarianceMatrix),
  /// Sends timer values to RECO.
  TimerValues(TimerValues),
  /// Sends altimeter offsets to RECO.
  AltimeterOffsets(AltimeterOffsets),
}

/// EKF process noise matrix (12x12 represented as four 3x3 submatrices).
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProcessNoiseMatrix {
  /// Gyro covariance matrix (row-major 3x3)
  pub nu_gv_mat: [f32; 9],
  /// Gyro bias covariance matrix (row-major 3x3)
  pub nu_gu_mat: [f32; 9],
  /// Accelerometer covariance matrix (row-major 3x3)
  pub nu_av_mat: [f32; 9],
  /// Accelerometer bias covariance matrix (row-major 3x3)
  pub nu_au_mat: [f32; 9],
}

/// Measurement noise matrix sent to RECO for EKF updates.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct MeasurementNoiseMatrix {
  /// GPS noise matrix (row-major 3x3)
  pub gps_noise_matrix: [f32; 9],
  /// Barometer noise term
  pub barometer_noise: f32,
}

/// Initial EKF state vector sent to RECO.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct EkfStateVector {
  /// Attitude of vehicle (quaternion)
  pub quaternion: [f32; 4],
  /// Position of vehicle in longitude, latitude, altitude frame
  pub lla_pos: [f32; 3],
  /// Velocity of vehicle
  pub velocity: [f32; 3],
  /// Gyroscope bias offset
  pub g_bias: [f32; 3],
  /// Accelerometer bias offset
  pub a_bias: [f32; 3],
  /// Gyro scale factor
  pub g_sf: [f32; 3],
  /// Acceleration scale factor
  pub a_sf: [f32; 3],
}

/// Initial covariance diagonal entries sent to RECO.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct InitialCovarianceMatrix {
  /// Uncertainty (variance) in attitude
  pub att_unc0: [f32; 3],
  /// Uncertainty (variance) in position
  pub pos_unc0: [f32; 3],
  /// Uncertainty (variance) in velocity
  pub vel_unc0: [f32; 3],
  /// Uncertainty (variance) in gyro bias
  pub gbias_unc0: [f32; 3],
  /// Uncertainty (variance) in accelerometer bias
  pub abias_unc0: [f32; 3],
  /// Uncertainty (variance) in gyro scale factor
  pub gsf_unc0: [f32; 3],
  /// Uncertainty (variance) in acceleration scale factor
  pub asf_unc0: [f32; 3],
}

/// Timer configuration sent to RECO.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct TimerValues {
  /// Number of seconds after launch to deploy drogue
  pub drouge_timer: f32,
  /// Number of seconds after launch to deploy main
  pub main_timer: f32,
  /// If true, use timer instead of EKF for drogue
  pub drouge_timer_enable: u8,
  /// If true, use timer instead of altimeter for main
  pub main_timer_enable: u8,
}

/// Altimeter offset configuration sent to RECO.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct AltimeterOffsets {
  /// EKF lockout time in milliseconds.
  pub ekf_lockout_time: u32,
  /// Altitude offset.
  pub h_offset_alt: f32,
  /// Altitude offset filter parameter.
  pub h_offset_filter: f32,
  /// Flight barometer FMF parameter.
  pub flight_baro_fmf_parameter: f32,
  /// Ground barometer FMF parameter.
  pub ground_baro_fmf_parameter: f32,
  /// Flight GPS FMF parameter.
  pub flight_gps_fmf_parameter: f32,
  /// Ground GPS FMF parameter.
  pub ground_gps_fmf_parameter: f32,
}
