// Ranging commands API

use crate::status::Status;

/// Defines how many of the 4 bytes of the address are checked against the request address sent by the initiator. Checked bytes are the LSB if check_length<4
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CheckLength {
    Addr8b = 1,
    Addr16b = 2,
    Addr24b = 3,
    Addr32b = 4,
}

/// Type of ranging result to return
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangingResKind {
    LatestRaw = 0,
    ExtendedRaw = 1,
    GainSteps = 2,
}

/// Sets the ranging Id for this device (used by the responder)
pub fn set_ranging_addr_cmd(addr: u32, check_length: CheckLength) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x78;

    cmd[2] |= ((addr >> 24) & 0xFF) as u8;
    cmd[3] |= ((addr >> 16) & 0xFF) as u8;
    cmd[4] |= ((addr >> 8) & 0xFF) as u8;
    cmd[5] |= (addr & 0xFF) as u8;
    cmd[6] |= (check_length as u8) & 0x7;
    cmd
}

/// Sets the ranging Id for the requests (used by the initiator)
pub fn set_ranging_req_addr_cmd(req_addr: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x79;

    cmd[2] |= ((req_addr >> 24) & 0xFF) as u8;
    cmd[3] |= ((req_addr >> 16) & 0xFF) as u8;
    cmd[4] |= ((req_addr >> 8) & 0xFF) as u8;
    cmd[5] |= (req_addr & 0xFF) as u8;
    cmd
}

/// Gets the ranging result (For Master or spy only). Based on type parameter, different results are returned. The Distance (m) = rng1*150/(2^12*LoraBW), with LoraBW in MHz. For extended mode type=1 results, rng1 and rng2 values should be averaged to get a distance estimation unaffected by Doppler effect
pub fn get_ranging_result_req(ranging_res_kind: RangingResKind) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x7A;

    cmd[2] |= ranging_res_kind as u8;
    cmd
}





/// Sets the Tx->Rx delay for the ranging calibration
pub fn set_ranging_tx_rx_delay_cmd(delay: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x7B;

    cmd[2] |= ((delay >> 24) & 0xFF) as u8;
    cmd[3] |= ((delay >> 16) & 0xFF) as u8;
    cmd[4] |= ((delay >> 8) & 0xFF) as u8;
    cmd[5] |= (delay & 0xFF) as u8;
    cmd
}

/// Sets the ranging specific parameters
pub fn set_ranging_params_cmd(extended: bool, spy_mode: bool, nb_symbols: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x7C;

    if extended { cmd[2] |= 128; }
    if spy_mode { cmd[2] |= 64; }
    cmd[2] |= nb_symbols & 0x3F;
    cmd
}

/// Gets the ranging counters for ranging exchanges. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats. Note: for extended ranging mode, the counters are incremented twice, once for each request/response
pub fn get_ranging_stats_req() -> [u8; 2] {
    [0x02, 0x7D]
}

// Response structs

/// Response for GetRangingResult command
#[derive(Default)]
pub struct RangingResultRsp([u8; 6]);

impl RangingResultRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Ranging measurement value. Distance in meter is given by rng*150/(2^12*Bandwidth),
    pub fn rng(&self) -> i32 {
        let raw = (self.0[4] as u32) |
            ((self.0[3] as u32) << 8) |
            ((self.0[2] as u32) << 16);
        raw as i32 - if (self.0[2] & 0x80) != 0 {1<<24} else {0}
    }

    /// RSSI value
    pub fn rssi(&self) -> u8 {
        self.0[5]
    }
}

impl AsMut<[u8]> for RangingResultRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRangingExtResult command
#[derive(Default)]
pub struct RangingExtResultRsp([u8; 9]);

impl RangingExtResultRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// First ranging measurement value
    pub fn rng1(&self) -> i32 {
        let raw = (self.0[4] as u32) |
            ((self.0[3] as u32) << 8) |
            ((self.0[2] as u32) << 16);
        raw as i32 - if (self.0[2] & 0x80) != 0 {1<<24} else {0}
    }

    /// RSSI value for first ranging measurement
    pub fn rssi1(&self) -> u8 {
        self.0[5]
    }

    /// Second ranging measurement value
    pub fn rng2(&self) -> i32 {
        let raw = (self.0[8] as u32) |
            ((self.0[7] as u32) << 8) |
            ((self.0[6] as u32) << 16);
        raw as i32 - if (self.0[6] & 0x80) != 0 {1<<24} else {0}
    }
}

impl AsMut<[u8]> for RangingExtResultRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRangingGainStep command
#[derive(Default)]
pub struct RangingGainStepRsp([u8; 4]);

impl RangingGainStepRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Radio gain used for first measurement
    pub fn gain1(&self) -> u8 {
        self.0[2]
    }

    /// Radio gain used for second measurement (if available)
    pub fn gain2(&self) -> u8 {
        self.0[3]
    }
}

impl AsMut<[u8]> for RangingGainStepRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRangingStats command
#[derive(Default)]
pub struct RangingStatsRsp([u8; 12]);

impl RangingStatsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Number of valid ranging exchanges
    pub fn exchange_valid(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Number of valid ranging requests
    pub fn request_valid(&self) -> u16 {
        (self.0[5] as u16) |
        ((self.0[4] as u16) << 8)
    }

    /// Number of completed responses
    pub fn response_done(&self) -> u16 {
        (self.0[7] as u16) |
        ((self.0[6] as u16) << 8)
    }

    /// Number of timeouts (For Initiator role: no response received from responder; For Responder role: no extended request received in extended mode)
    pub fn timeout(&self) -> u16 {
        (self.0[9] as u16) |
        ((self.0[8] as u16) << 8)
    }

    /// Number of discarded requests
    pub fn request_discarded(&self) -> u16 {
        (self.0[11] as u16) |
        ((self.0[10] as u16) << 8)
    }
}

impl AsMut<[u8]> for RangingStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
