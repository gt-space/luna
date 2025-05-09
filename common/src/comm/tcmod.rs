use super::{flight::Ingestible, VehicleState};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

type Temperature = f64;
#[derive(
    Copy, Clone, Default, MaxSize, Debug, Deserialize, PartialEq, Serialize,
)]
pub struct Tc {
    pub temperatures: [[Temperature, 3;]; 4]
}

#[derive(
    Copy, Clone, Default, MaxSize, Debug, Deserialize, PartialEq, Serialize,
)]
pub struct DataPoint {
    pub state: Tc,
    pub timestamp: f64
}

impl Ingestible for DataPoint{
    fn ingest(&self, vehicle_state: &mut VehicleState) {
        vehicle_state.tcmod = self.state;
    }
}

