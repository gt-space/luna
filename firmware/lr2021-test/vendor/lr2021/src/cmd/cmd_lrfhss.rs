// Lrfhss commands API


/// Coding rate selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LrfhssCr {
    Cr5p6 = 0,
    Cr2p3 = 1,
    Cr1p2 = 2,
    Cr1p3 = 3,
}

/// Frequency grid selection (25.39kHz or 3.91kHz)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Grid {
    Grid25 = 0,
    Grid4 = 1,
}

/// Hopping mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hopping {
    NoHopping = 0,
    Hopping = 1,
    TestPayloadEncoded = 2,
    TestPaRamp = 3,
}

/// Bandwidth occupied by hopping pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LrfhssBw {
    Bw39p06 = 0,
    Bw85p94 = 1,
    Bw136p72 = 2,
    Bw183p59 = 3,
    Bw335p94 = 4,
    Bw386p72 = 5,
    Bw722p66 = 6,
    Bw773p44 = 7,
    Bw1523p4 = 8,
    Bw1574p2 = 9,
}

/// Prepare LR-FHSS packet in the FIFO without sending it
pub fn lr_fhss_build_frame_cmd(sync_header_cnt: u8, lrfhss_cr: LrfhssCr, grid: Grid, hopping: Hopping, lrfhss_bw: LrfhssBw, sequence: u16, offset: i8) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x56;

    cmd[2] |= sync_header_cnt & 0xF;
    cmd[2] |= ((lrfhss_cr as u8) & 0xF) << 4;
    cmd[3] |= ((grid as u8) & 0xF) << 4;
    cmd[4] |= (hopping as u8) & 0xF;
    cmd[4] |= ((lrfhss_bw as u8) & 0xF) << 4;
    cmd[5] |= ((sequence >> 8) & 0xFF) as u8;
    cmd[6] |= (sequence & 0xFF) as u8;
    cmd[7] |= (offset) as u8;
    cmd
}

/// Sets the LR-FHSS syncword. Reset value is { 0x2C, 0x0F, 0x79, 0x95 }
pub fn set_lr_fhss_sync_word_cmd(syncword: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x57;

    cmd[2] |= ((syncword >> 24) & 0xFF) as u8;
    cmd[3] |= ((syncword >> 16) & 0xFF) as u8;
    cmd[4] |= ((syncword >> 8) & 0xFF) as u8;
    cmd[5] |= (syncword & 0xFF) as u8;
    cmd
}

/// Writes the LR-FHSS hopping table.
pub fn write_lr_fhss_hopping_table_cmd(hop_en: bool, freq_hz: bool, pkt_length: u16, nb_used_freqs: u8, nb_hopping_blocks: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x59;

    if hop_en { cmd[2] |= 1; }
    if freq_hz { cmd[2] |= 128; }
    cmd[3] |= ((pkt_length >> 8) & 0xFF) as u8;
    cmd[4] |= (pkt_length & 0xFF) as u8;
    cmd[5] |= nb_used_freqs;
    cmd[6] |= nb_hopping_blocks;
    cmd
}
