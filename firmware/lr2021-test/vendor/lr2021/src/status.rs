//! # Device status and interrupts
//!
//! The device status is represented by two bytes provided on each command.
//! It contains:
//! - Information about previous command (Ok, fail, error, ...)
//! - A flag for interrupt pending
//! - The source of the last reset (manual, analog, watchdog, ...)
//! - Current chip Mode (Sleep, Standby, Tx, RX, ...)
//!
//! The interrupt structure `Intr` allows to both configrue which interrupt should be assigned to a pin
//! with the command [`set_dio_irq`](crate::Lr2021::set_dio_irq) and easily get which interrupt is currently raised
//! after a [`get_status`](crate::Lr2021::get_status) or [`get_and_clear_irq`](crate::Lr2021::get_and_clear_irq).

use super::Lr2021Error;

/// Status sent at the beginning of each SPI command
///  - 11:9 = Command status
///  -    8 Interrupt pending
///  -  7:4 Reset source
///  -  2:0 Chip Mode
#[derive(Default, Clone, Copy)]
pub struct Status(u16);

/// Command status
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CmdStatus {
    Fail = 0, // Last Command could not be executed
    PErr = 1, // Last command had invalid parameters or the OpCode is unknown
    Ok   = 2, // Last Command succeed
    Data = 3, // Last command succeed and now streaming data
    Unknown = 8, // Unknown status
}

impl From<u8> for CmdStatus {
    fn from(value: u8) -> Self {
        match value {
            0 => CmdStatus::Fail,
            1 => CmdStatus::PErr,
            2 => CmdStatus::Ok,
            3 => CmdStatus::Data,
            _ => CmdStatus::Unknown,
        }
    }
}

impl CmdStatus {
    /// Check command status and return Ok/Err
    pub fn check(&self) -> Result<(), Lr2021Error> {
        match self {
            CmdStatus::Unknown => Err(Lr2021Error::Unknown),
            CmdStatus::Fail => Err(Lr2021Error::CmdFail),
            CmdStatus::PErr => Err(Lr2021Error::CmdErr),
            CmdStatus::Ok   |
            CmdStatus::Data => Ok(()),
        }
    }
}


/// Reset Source
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetSrc {
    Cleared = 0,
    Analog = 1,
    External = 2,
    Watchdog = 4,
    Iocd = 5,
    Rtc = 6,
    Unknown = 16, // Unknown Source
}

/// Chip Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipModeStatus {
    Sleep = 0,
    Rc    = 1,
    Xosc  = 2,
    Fs    = 3,
    Rx    = 4,
    Tx    = 5,
    Unknown = 8, // Unknown Mode
}

impl Status {

    /// Create a status from a slice of at least two elements
    pub fn from_array(bytes: [u8;2]) -> Status {
        let v = u16::from_be_bytes(bytes);
        Status(v)
    }

    /// Create a status from a slice of at least two elements
    pub fn from_slice(bytes: &[u8]) -> Status {
        let v = ((*bytes.first().unwrap_or(&0) as u16) << 8)
            | (*bytes.get(1).unwrap_or(&0) as u16);
        Status(v)
    }

    /// Return Command status
    pub fn cmd(&self) -> CmdStatus {
        let bits_cmd = ((self.0 >> 9) & 7) as u8;
        bits_cmd.into()
    }

    pub fn is_ok(&self) -> bool {
        matches!(self.cmd(),CmdStatus::Ok | CmdStatus::Data)
    }

    /// Return true if an Interrupt is pending
    pub fn irq(&self) -> bool {
        (self.0 & 0x0100) != 0
    }

    /// Return source of last reset
    pub fn reset_src(&self) -> ResetSrc {
        let bits_rst = (self.0 >> 4) & 15;
        match bits_rst {
            0 => ResetSrc::Cleared,
            1 => ResetSrc::Analog,
            2 => ResetSrc::External,
            4 => ResetSrc::Watchdog,
            5 => ResetSrc::Iocd,
            6 => ResetSrc::Rtc,
            _ => ResetSrc::Unknown
        }
    }

    /// Return source of last reset
    pub fn chip_mode(&self) -> ChipModeStatus {
        let bits_mode = self.0 & 7;
        match bits_mode {
            0 => ChipModeStatus::Sleep,
            1 => ChipModeStatus::Rc,
            2 => ChipModeStatus::Xosc,
            3 => ChipModeStatus::Fs,
            4 => ChipModeStatus::Rx,
            5 => ChipModeStatus::Tx,
            _ => ChipModeStatus::Unknown,
        }
    }

    /// Check command status and return Ok/Err
    pub fn check(&self) -> Result<(), Lr2021Error> {
        self.cmd().check()
    }

}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, fmt: defmt::Formatter) {
        match self.cmd() {
            CmdStatus::Fail    => defmt::write!(fmt, "Command failed !"),
            CmdStatus::PErr    => defmt::write!(fmt, "Illegal parameters"),
            CmdStatus::Unknown => defmt::write!(fmt, "Invalid status"),
            CmdStatus::Ok |
            CmdStatus::Data    => {
                defmt::write!(fmt, "Command succeded");
                if self.irq() {
                    defmt::write!(fmt, " | IRQ pending");
                }
                let rst = self.reset_src();
                if rst!=ResetSrc::Cleared {
                    defmt::write!(fmt, " | Reset from {}", rst);
                }
                defmt::write!(fmt, " | Chip in {}", self.chip_mode());
            }
        }
    }
}

/// RX FIFO threshold reached
pub const IRQ_MASK_RX_FIFO             : u32 = 0x00000001;
/// TX FIFO threshold reached
pub const IRQ_MASK_TX_FIFO             : u32 = 0x00000002;
/// Valid ranging request received (slave)
pub const IRQ_MASK_RNG_REQ_VLD         : u32 = 0x00000004;
/// IRq for time-stamping end of packet Tx, without dependent delay of mode switching.
/// Only to be used for timestamping, not for changing mode or re-configuring the device.
pub const IRQ_MASK_TX_TIMESTAMP        : u32 = 0x00000008;
/// IRq for time-stamping end of packet Rx, without dependent delay of demodulation or mode switching.
/// Only to be used for timestamping, not for changing mode or re-configuring the device.
pub const IRQ_MASK_RX_TIMESTAMP        : u32 = 0x00000010;
/// Preamble detected
pub const IRQ_MASK_PREAMBLE_DETECTED   : u32 = 0x00000020;
/// LoRa header detected / Valid sync word
pub const IRQ_MASK_HEADER_VALID        : u32 = 0x00000040;
/// Channel activity detected
pub const IRQ_MASK_CAD_DETECTED        : u32 = 0x00000080;

/// LoRa header precise timestamp (explicit mode). In implicit mode will assert after 8 symbols of the payload.
pub const IRQ_MASK_LORA_HDR_TIMESTAMP  : u32 = 0x00000100;
/// LoRa header CRC error
pub const IRQ_MASK_HEADER_ERR          : u32 = 0x00000200;
/// End of life
pub const IRQ_MASK_EOL                 : u32 = 0x00000400;
/// PA OCP/OVP was triggered
pub const IRQ_MASK_PA                  : u32 = 0x00000800;
/// IRq for LoRa intra-packet hopping
pub const IRQ_MASK_LORA_TX_RX_HOP      : u32 = 0x00001000;
/// Syncword match failed after detection (for debug)
pub const IRQ_MASK_SYNC_FAIL           : u32 = 0x00002000;
/// End of LoRa symbol (for debug)
pub const IRQ_MASK_LORA_SYMBOL_END     : u32 = 0x00004000;
/// New statistics available in timestamp register (debug)
pub const IRQ_MASK_LORA_TIMESTAMP_STAT : u32 = 0x00008000;

/// An error other than a command error occurred (See GetErrors)
pub const IRQ_MASK_ERROR               : u32 = 0x00010000;
/// There was a host command fail/error
pub const IRQ_MASK_CMD                 : u32 = 0x00020000;
/// Packet received
pub const IRQ_MASK_RX_DONE             : u32 = 0x00040000;
/// Packet transmission completed
pub const IRQ_MASK_TX_DONE             : u32 = 0x00080000;
/// Channel activity detection finished
pub const IRQ_MASK_CAD_DONE            : u32 = 0x00100000;
/// Rx or Tx timeout
pub const IRQ_MASK_TIMEOUT             : u32 = 0x00200000;
/// IRq raised if the packet was received with a wrong CRC
pub const IRQ_MASK_CRC_ERROR           : u32 = 0x00400000;
/// IRq raised if the packet was received with a length error
pub const IRQ_MASK_LEN_ERROR           : u32 = 0x00800000;

/// IRq raised if the packet was received with a wrong address match
pub const IRQ_MASK_ADDR_ERROR          : u32 = 0x01000000;
/// IRQ after each ramp-up for intra-packet hopping
pub const IRQ_MASK_FHSS                : u32 = 0x02000000;
/// Host can load new frequencies table
pub const IRQ_MASK_INTER_PACKET1       : u32 = 0x04000000;
/// Host can load new payload
pub const IRQ_MASK_INTER_PACKET2       : u32 = 0x08000000;
/// Slave sent its ranging response
pub const IRQ_MASK_RNG_RESP_DONE       : u32 = 0x10000000;
/// Ranging request discarded (no address match)
pub const IRQ_MASK_RNG_REQ_DIS         : u32 = 0x20000000;
/// Master received a valid ranging response from the slave
pub const IRQ_MASK_RNG_EXCH_VLD        : u32 = 0x40000000;
/// Master did not receive a response from the slave
pub const IRQ_MASK_RNG_TIMEOUT         : u32 = 0x80000000;

/// Mask to enable all interrupt usefull for LoRa TX/RX (preamble detected, header ok/err, tx/rx done, timeout, CRC error)
pub const IRQ_MASK_LORA_TXRX : u32 =
    IRQ_MASK_PREAMBLE_DETECTED |
    IRQ_MASK_HEADER_ERR | IRQ_MASK_HEADER_VALID |
    IRQ_MASK_RX_DONE | IRQ_MASK_TX_DONE |
    IRQ_MASK_CAD_DETECTED | IRQ_MASK_CAD_DONE |
    IRQ_MASK_TIMEOUT | IRQ_MASK_CRC_ERROR;

/// Mask to enable all interrupt usefull for FSK TX/RX (preamble detected, tx/rx done, timeout, CRC/Length error)
pub const IRQ_MASK_FSK_TXRX : u32 =
    IRQ_MASK_PREAMBLE_DETECTED |
    IRQ_MASK_RX_DONE | IRQ_MASK_TX_DONE |
    IRQ_MASK_LEN_ERROR |
    IRQ_MASK_TIMEOUT | IRQ_MASK_CRC_ERROR;

#[derive(Default, Clone, Copy)]
pub struct Intr(u32);

impl Intr {

    /// Create Interrupt status from a slice
    /// Handle gracefully case where slice is smaller than interrupt size
    /// (this happens when retrieving interrupt value while writing command smaller than 6B)
    pub fn from_slice(bytes: &[u8]) -> Intr {
        let v = ((*bytes.first().unwrap_or(&0) as u32) << 24)
            | ((*bytes.get(1).unwrap_or(&0) as u32) << 16)
            | ((*bytes.get(2).unwrap_or(&0) as u32) <<  8)
            | (*bytes.get(3).unwrap_or(&0) as u32);
        Intr(v)
    }

    /// Create a new interrupt using a mask value
    /// Use IRQ_MASK_* constant to build it
    pub fn new(value: u32) -> Intr {
        Intr(value)
    }

    /// Create a new interrupt to raise IRQ on TX/RX Done as-well as Timeout error
    pub fn new_txrx() -> Intr {
        Intr(IRQ_MASK_RX_DONE|IRQ_MASK_TX_DONE|IRQ_MASK_TIMEOUT)
    }

    /// Create a new interrupt for LoRa Ranging operations
    /// Enable Ranging exchange valid, response done, request discarded and timeout
    pub fn new_ranging() -> Intr {
        Intr(IRQ_MASK_RNG_EXCH_VLD|IRQ_MASK_RNG_RESP_DONE|IRQ_MASK_RNG_REQ_DIS|IRQ_MASK_TIMEOUT|IRQ_MASK_RNG_TIMEOUT)
    }

    /// Return the interrupt status as u32
    pub fn value(&self) -> u32 {
        self.0
    }

    /// Check if the interrupt status
    pub fn intr_match(&self, mask: u32) -> bool {
        self.value() & mask != 0
    }

    pub fn none(&self) -> bool {
        self.0 == 0
    }

    /// Returns true if RX FIFO threshold reached interrupt has been raised
    pub fn rx_fifo(&self) -> bool {
        (self.0 & IRQ_MASK_RX_FIFO) != 0
    }
    /// Returns true if TX FIFO threshold reached interrupt has been raised
    pub fn tx_fifo(&self) -> bool {
        (self.0 & IRQ_MASK_TX_FIFO) != 0
    }
    /// Returns true if valid ranging request received (slave) interrupt has been raised
    pub fn rng_req_vld(&self) -> bool {
        (self.0 & IRQ_MASK_RNG_REQ_VLD) != 0
    }
    /// Returns true if TX timestamp interrupt has been raised.
    /// IRq for time-stamping end of packet Tx, without dependent delay of mode switching.
    /// Only to be used for timestamping, not for changing mode or re-configuring the device.
    pub fn tx_timestamp(&self) -> bool {
        (self.0 & IRQ_MASK_TX_TIMESTAMP) != 0
    }
    /// Returns true if RX timestamp interrupt has been raised.
    /// IRq for time-stamping end of packet Rx, without dependent delay of demodulation or mode switching.
    /// Only to be used for timestamping, not for changing mode or re-configuring the device.
    pub fn rx_timestamp(&self) -> bool {
        (self.0 & IRQ_MASK_RX_TIMESTAMP) != 0
    }
    /// Returns true if preamble detected interrupt has been raised
    pub fn preamble_detected(&self) -> bool {
        (self.0 & IRQ_MASK_PREAMBLE_DETECTED) != 0
    }
    /// Returns true if LoRa header detected / valid sync word interrupt has been raised
    pub fn header_valid(&self) -> bool {
        (self.0 & IRQ_MASK_HEADER_VALID) != 0
    }
    /// Returns true if channel activity detected interrupt has been raised
    pub fn cad_detected(&self) -> bool {
        (self.0 & IRQ_MASK_CAD_DETECTED) != 0
    }
    /// Returns true if LoRa header precise timestamp interrupt has been raised.
    /// In explicit mode: LoRa header precise timestamp. In implicit mode will assert after 8 symbols of the payload.
    pub fn lora_hdr_timestamp(&self) -> bool {
        (self.0 & IRQ_MASK_LORA_HDR_TIMESTAMP) != 0
    }
    /// Returns true if LoRa header CRC error interrupt has been raised
    pub fn header_err(&self) -> bool {
        (self.0 & IRQ_MASK_HEADER_ERR) != 0
    }
    /// Returns true if end of life interrupt has been raised
    pub fn eol(&self) -> bool {
        (self.0 & IRQ_MASK_EOL) != 0
    }
    /// Returns true if PA OCP/OVP was triggered interrupt has been raised
    pub fn pa(&self) -> bool {
        (self.0 & IRQ_MASK_PA) != 0
    }
    /// Returns true if LoRa intra-packet hopping interrupt has been raised
    pub fn lora_tx_rx_hop(&self) -> bool {
        (self.0 & IRQ_MASK_LORA_TX_RX_HOP) != 0
    }
    /// Returns true if syncword match failed after detection interrupt has been raised (for debug)
    pub fn sync_fail(&self) -> bool {
        (self.0 & IRQ_MASK_SYNC_FAIL) != 0
    }
    /// Returns true if end of LoRa symbol interrupt has been raised (for debug)
    pub fn lora_symbol_end(&self) -> bool {
        (self.0 & IRQ_MASK_LORA_SYMBOL_END) != 0
    }
    /// Returns true if new statistics available in timestamp register interrupt has been raised (debug)
    pub fn lora_timestamp_stat(&self) -> bool {
        (self.0 & IRQ_MASK_LORA_TIMESTAMP_STAT) != 0
    }
    /// Returns true if an error other than a command error occurred interrupt has been raised (See GetErrors)
    pub fn error(&self) -> bool {
        (self.0 & IRQ_MASK_ERROR) != 0
    }
    /// Returns true if there was a host command fail/error interrupt has been raised
    pub fn cmd(&self) -> bool {
        (self.0 & IRQ_MASK_CMD) != 0
    }
    /// Returns true if packet received interrupt has been raised
    pub fn rx_done(&self) -> bool {
        (self.0 & IRQ_MASK_RX_DONE) != 0
    }
    /// Returns true if packet transmission completed interrupt has been raised
    pub fn tx_done(&self) -> bool {
        (self.0 & IRQ_MASK_TX_DONE) != 0
    }
    /// Returns true if channel activity detection finished interrupt has been raised
    pub fn cad_done(&self) -> bool {
        (self.0 & IRQ_MASK_CAD_DONE) != 0
    }
    /// Returns true if Rx or Tx timeout interrupt has been raised
    pub fn timeout(&self) -> bool {
        (self.0 & IRQ_MASK_TIMEOUT) != 0
    }
    /// Returns true if the packet was received with a wrong CRC interrupt has been raised
    pub fn crc_error(&self) -> bool {
        (self.0 & IRQ_MASK_CRC_ERROR) != 0
    }
    /// Returns true if the packet was received with a length error interrupt has been raised
    pub fn len_error(&self) -> bool {
        (self.0 & IRQ_MASK_LEN_ERROR) != 0
    }
    /// Returns true if the packet was received with a wrong address match interrupt has been raised
    pub fn addr_error(&self) -> bool {
        (self.0 & IRQ_MASK_ADDR_ERROR) != 0
    }
    /// Returns true if IRQ after each ramp-up for intra-packet hopping interrupt has been raised
    pub fn fhss(&self) -> bool {
        (self.0 & IRQ_MASK_FHSS) != 0
    }
    /// Returns true if host can load new frequencies table interrupt has been raised
    pub fn inter_packet1(&self) -> bool {
        (self.0 & IRQ_MASK_INTER_PACKET1) != 0
    }
    /// Returns true if host can load new payload interrupt has been raised
    pub fn inter_packet2(&self) -> bool {
        (self.0 & IRQ_MASK_INTER_PACKET2) != 0
    }
    /// Returns true if slave sent its ranging response interrupt has been raised
    pub fn rng_resp_done(&self) -> bool {
        (self.0 & IRQ_MASK_RNG_RESP_DONE) != 0
    }
    /// Returns true if ranging request discarded (no address match) interrupt has been raised
    pub fn rng_req_dis(&self) -> bool {
        (self.0 & IRQ_MASK_RNG_REQ_DIS) != 0
    }
    /// Returns true if master received a valid ranging response from the slave interrupt has been raised
    pub fn rng_exch_vld(&self) -> bool {
        (self.0 & IRQ_MASK_RNG_EXCH_VLD) != 0
    }
    /// Returns true if master did not receive a response from the slave interrupt has been raised
    pub fn rng_timeout(&self) -> bool {
        (self.0 & IRQ_MASK_RNG_TIMEOUT) != 0
    }

}

impl From<u32> for Intr {
    fn from(value: u32) -> Self {
        Intr::new(value)
    }
}


#[cfg(feature = "defmt")]
impl defmt::Format for Intr {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Intr: ");
        if self.none() {
            defmt::write!(f, "None");
            return;
        }
        if self.error()               {defmt::write!(f, "Error ")};
        if self.cmd()                 {defmt::write!(f, "CmdError ")};
        if self.rx_fifo()             {defmt::write!(f, "FifoRx ")};
        if self.tx_fifo()             {defmt::write!(f, "FifoTx ")};
        if self.eol()                 {defmt::write!(f, "EndOfLife ")};
        if self.pa()                  {defmt::write!(f, "PowerAmplifier ")};
        if self.preamble_detected()   {defmt::write!(f, "PreambleDetected ")};
        if self.sync_fail()           {defmt::write!(f, "SyncFail ")};
        if self.cad_detected()        {defmt::write!(f, "CadDetected ")};
        if self.timeout()             {defmt::write!(f, "Timeout ")};
        if self.crc_error()           {defmt::write!(f, "CrcError ")};
        if self.len_error()           {defmt::write!(f, "LenError ")};
        if self.addr_error()          {defmt::write!(f, "AddrError ")};
        if self.header_valid()        {defmt::write!(f, "HeaderValid ")};
        if self.header_err()          {defmt::write!(f, "HeaderError ")};
        if self.lora_tx_rx_hop()      {defmt::write!(f, "LoraTxRxHop ")};
        if self.lora_symbol_end()     {defmt::write!(f, "LoraSymbolEnd ")};
        if self.rx_done()             {defmt::write!(f, "RxDone ")};
        if self.tx_done()             {defmt::write!(f, "TxDone ")};
        if self.cad_done()            {defmt::write!(f, "CadDone ")};
        if self.tx_timestamp()        {defmt::write!(f, "TimestampTx ")};
        if self.rx_timestamp()        {defmt::write!(f, "TimestampRx ")};
        if self.lora_hdr_timestamp()  {defmt::write!(f, "TimestampLoraHeader ")};
        if self.lora_timestamp_stat() {defmt::write!(f, "TimestampLoraStat ")};
        if self.fhss()                {defmt::write!(f, "FHSS ")};
        if self.inter_packet1()       {defmt::write!(f, "InterPacket1 ")};
        if self.inter_packet2()       {defmt::write!(f, "InterPacket2 ")};
        if self.rng_resp_done()       {defmt::write!(f, "RangingRespDone ")};
        if self.rng_req_vld()         {defmt::write!(f, "RangingReqValid ")};
        if self.rng_req_dis()         {defmt::write!(f, "RangingReqDis ")};
        if self.rng_exch_vld()        {defmt::write!(f, "RangingExchValid ")};
        if self.rng_timeout()         {defmt::write!(f, "RangingTimeout")};
    }
}