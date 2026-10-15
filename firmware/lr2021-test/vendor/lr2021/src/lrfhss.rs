//! # API related to LR-FHSS operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for LR-FHSS (Long Range Frequency Hopping Spread Spectrum) modulation.
//! LR-FHSS is a modulation scheme designed for long-range, low-power communication with high interference resistance through frequency hopping.
//! It is particularly useful for regulatory compliance (e.g., FCC) and applications requiring robust communication in noisy environments.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for LR-FHSS operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::lrfhss::{LrfhssCr, Grid, Hopping, LrfhssBw};
//!
//! // Set packet type to LR-FHSS
//! lr2021.set_packet_type(PacketType::LrFhss).await.expect("Setting packet type");
//!
//! // Configure syncword (default is 0x2C0F7995)
//! lr2021.set_lrfhss_syncword(0x2C0F7995).await.expect("Setting syncword");
//!
//! // Build LR-FHSS packet with payload
//! let payload = b"Hello, LR-FHSS!";
//! lr2021.lrfhss_build_packet(
//!     1,                      // Sync header count
//!     LrfhssCr::Cr5p6,        // Coding rate: 5/6
//!     Grid::Grid25,           // Frequency grid: 25.39kHz
//!     Hopping::HoppingEnabled, // Enable intra-packet hopping
//!     LrfhssBw::Bw1523p4,     // Bandwidth: 1523.4kHz (FCC use case)
//!     0,                      // Hop sequence
//!     0,                      // Device Frequency offset
//!     pld                     // Payload
//! ).await.expect("Building LR-FHSS packet");
//!
//! // Transmit the packet\
//! lr2021.set_tx(0).await.expect("Starting transmission");
//! ```
//!
//! ## Available Methods
//!
//! - [`lrfhss_build_packet`](Lr2021::lrfhss_build_packet) - Encode payload and configure internal hopping table for LR-FHSS transmission
//! - [`set_lrfhss_syncword`](Lr2021::set_lrfhss_syncword) - Configure LR-FHSS syncword (4 bytes, default: 0x2C0F7995)
//! - [`set_lrfhss_hopping`](Lr2021::set_lrfhss_hopping) - Configure LR-FHSS hopping table

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_lrfhss::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Clone)]
pub struct LrfhssHop {
    /// Frequency
    freq: u32,
    /// Duration of the hop in number of symbol
    len: u16,
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    // TODO: add dedicated struct and find a good default set of values (maybe 2-3 builder method)
    #[allow(clippy::too_many_arguments)]
    /// Prepare the LR-FHSS packet
    pub async fn lrfhss_build_packet(&mut self, sync_header_cnt: u8, cr: LrfhssCr, grid: Grid, hopping: Hopping, bw: LrfhssBw, sequence: u16, offset: i8, pld: &[u8]) -> Result<(), Lr2021Error> {
        let req = lr_fhss_build_frame_cmd(sync_header_cnt, cr, grid, hopping, bw, sequence, offset);
        self.cmd_data_wr(&req, pld).await
    }

    /// Configure Syncword of LRFHSS packet
    /// Default value is 0x2C0F7995
    pub async fn set_lrfhss_syncword(&mut self, syncword: u32) -> Result<(), Lr2021Error> {
        let req = set_lr_fhss_sync_word_cmd(syncword);
        self.cmd_wr(&req).await
    }

    /// Set the LRFHSS hopping table
    /// The data parameter should be up to 40 pairs (freq (4B), Nb_symbols (2B))
    pub async fn set_lrfhss_hopping(&mut self, hop_en: bool, freq_hz: bool, pkt_length: u16, nb_used_freqs: u8, nb_hopping_blocks: u8, hops: &[LrfhssHop]) -> Result<(), Lr2021Error> {
        let req = write_lr_fhss_hopping_table_cmd(hop_en, freq_hz, pkt_length, nb_used_freqs, nb_hopping_blocks);
        self.cmd_wr_begin(&req).await?;
        for hop in hops {
            self.buffer_mut()[0] = ((hop.freq >> 24) & 0xFF) as u8;
            self.buffer_mut()[1] = ((hop.freq >> 16) & 0xFF) as u8;
            self.buffer_mut()[2] = ((hop.freq >> 8 ) & 0xFF) as u8;
            self.buffer_mut()[3] = ((hop.freq      ) & 0xFF) as u8;
            self.buffer_mut()[4] = ((hop.len >> 8) & 0xFF) as u8;
            self.buffer_mut()[5] = ((hop.len     ) & 0xFF) as u8;
            self.spi.transfer_in_place(&mut self.buffer.data_mut()[..6]).await
                .map_err(|_| Lr2021Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }


}