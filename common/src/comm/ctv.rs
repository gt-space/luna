#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct Vec3 {
  pub x: f64,
  pub y: f64,
  pub z: f64,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct Vec4 {
  pub w: f64,
  pub x: f64,
  pub y: f64,
  pub z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct ControlState {
  pub position: Vec3,
  pub velocity: Vec3,
  pub acceleration: Vec3,
  pub attitude: Vec4,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct ControlVector {
  pub thrust: f64,
  pub tvc_yaw: f64,
  pub tvc_pitch: f64,
  pub rcs_torque: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum CtvControlMessage {
  Control(ControlVector),
  Abort,
}
