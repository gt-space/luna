use std::collections::HashMap;

use serde::{Deserialize, Serialize};

/// Aggregated RBF status exposed to downstream telemetry consumers.
#[derive(
    Clone,
    Debug,
    Default,
    Deserialize,
    PartialEq,
    Serialize,
    rkyv::Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct RbfState {
    /// Latest BMS RBF reading, if any.
    pub bms: u8,

    /// RECO RBF reading for each MCU
    pub reco: [u8; 3],

    /// RBF state for each SAM board (board_id, rbf_value)
    pub sam: HashMap<String, u8>,
}
