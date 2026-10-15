//! # FIFO handling API
//!
//! This module provides general APIs to control and access the TX/RX FIFO
//!
//! ## Available Methods
//!
//! ### FIFO Interrupts
//! - [`set_fifo_irq_en`](Lr2021::set_fifo_irq_en) - Configure interrupts enable for TX/RX Fifo
//! - [`set_fifo_irq_cfg`](Lr2021::set_fifo_irq_cfg) - Configure interrupts for TX/RX Fifo (enable with low and high threshold)
//! - [`get_fifo_irq`](Lr2021::get_fifo_irq) - Return the irqs flag for TX and RX FIFO
//!
//! ### TX FIFO
//! - [`wr_tx_fifo_from`](Lr2021::wr_tx_fifo_from) - Write data to TX FIFO from external buffer
//! - [`wr_tx_fifo`](Lr2021::wr_tx_fifo) - Write data to TX FIFO from internal buffer
//! - [`get_tx_fifo_lvl`](Lr2021::get_tx_fifo_lvl) - Get number of bytes in TX FIFO
//! - [`clear_tx_fifo`](Lr2021::clear_tx_fifo) - Clear all data from TX FIFO
//!
//! ### RX FIFO  
//! - [`rd_rx_fifo_to`](Lr2021::rd_rx_fifo_to) - Read RX FIFO data to external buffer
//! - [`rd_rx_fifo`](Lr2021::rd_rx_fifo) - Read RX FIFO data to internal buffer
//! - [`get_rx_fifo_lvl`](Lr2021::get_rx_fifo_lvl) - Get number of bytes in RX FIFO
//! - [`clear_rx_fifo`](Lr2021::clear_rx_fifo) - Clear all data from RX FIFO

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use super::cmd::cmd_system::*;

use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Default, Clone, Copy)]
/// FIFO IRQ enable flags
pub struct FifoIrqEn(u8);

impl FifoIrqEn {

    /// All IRQ disabled
    pub fn none() -> Self {
        Self(0x00)
    }

    /// Enable High and low IRQ
    pub fn high_low() -> Self {
        Self(0x06)
    }

    /// All IRQ enabled
    pub fn all() -> Self {
        Self(0x3F)
    }

    /// Return the byte representation of the enable
    pub fn value(&self) -> u8 {
        self.0
    }

    /// Enable Fifo empty IRQ
    pub fn with_empty(&self) -> Self {
        Self(self.0 | 0x01)
    }

    /// Enable Fifo low IRQ
    pub fn with_low(&self) -> Self {
        Self(self.0 | 0x02)
    }

    /// Enable Fifo high IRQ
    pub fn with_high(&self) -> Self {
        Self(self.0 | 0x04)
    }

    /// Enable Fifo full IRQ
    pub fn with_full(&self) -> Self {
        Self(self.0 | 0x08)
    }

    /// Enable Fifo overflow IRQ
    pub fn with_overflow(&self) -> Self {
        Self(self.0 | 0x10)
    }

    /// Enable Fifo underflow IRQ
    pub fn with_underflow(&self) -> Self {
        Self(self.0 | 0x20)
    }

    /// Fifo empty IRQ enabled
    pub fn has_empty(&self) -> bool {
        self.0 & 0x01 !=0
    }

    /// Fifo low IRQ enabled
    pub fn has_low(&self) -> bool {
        self.0 & 0x02 !=0
    }

    /// Fifo high IRQ enabled
    pub fn has_high(&self) -> bool {
        self.0 & 0x04 !=0
    }

    /// Fifo full IRQ enabled
    pub fn has_full(&self) -> bool {
        self.0 & 0x08 !=0
    }

    /// Fifo overflow IRQ enabled
    pub fn has_overflow(&self) -> bool {
        self.0 & 0x10 !=0
    }

    /// Fifo underflow IRQ enabled
    pub fn has_underflow(&self) -> bool {
        self.0 & 0x20 !=0
    }

}

#[derive(Default, Clone, Copy)]
/// FIFO IRQ enable flags
pub struct FifoIrqCfg{
    pub en: FifoIrqEn,
    pub thr_low: u16,
    pub thr_high: u16,
}

impl FifoIrqCfg {
    /// Create configuration for FIFO IRQ (TX or RX)
    pub fn new(en: FifoIrqEn, thr_low: u16, thr_high: u16) -> Self {
        Self {en, thr_low, thr_high}
    }
}


impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{
    /// Configure interrupts enable for TX/RX Fifo
    pub async fn set_fifo_irq_en(&mut self, tx_en: FifoIrqEn, rx_en: FifoIrqEn) -> Result<(), Lr2021Error> {
        let req = config_fifo_irq_cmd(rx_en.value(), tx_en.value());
        self.cmd_wr(&req).await
    }

    /// Configure interrupts for TX/RX Fifo (enable with low and high threshold)
    pub async fn set_fifo_irq_cfg(&mut self, tx: FifoIrqCfg, rx: FifoIrqCfg) -> Result<(), Lr2021Error> {
        let req = config_fifo_irq_adv_cmd(rx.en.value(), tx.en.value(), rx.thr_high, tx.thr_low, rx.thr_low, tx.thr_high);
        self.cmd_wr(&req).await
    }

    /// Return the irqs flag for TX and RX FIFO
    pub async fn get_fifo_irq(&mut self) -> Result<(FifoIrqEn,FifoIrqEn), Lr2021Error> {
        let req = get_fifo_irq_flags_req();
        let mut rsp = FifoIrqFlagsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        let tx_flags = FifoIrqEn(rsp.tx_fifo_flags());
        let rx_flags = FifoIrqEn(rsp.rx_fifo_flags());
        Ok((tx_flags,rx_flags))
    }

    /// Write data to the TX FIFO
    /// Check number of bytes available with get_tx_fifo_lvl()
    pub async fn wr_tx_fifo_from(&mut self, buffer: &[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data_wr(&[0,2], buffer).await
    }

    /// Write data to the TX FIFO
    /// Check number of bytes available with get_tx_fifo_lvl()
    pub async fn wr_tx_fifo(&mut self, len: usize) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&[0,2]).await?;
        self.spi
            .transfer_in_place(&mut self.buffer.data_mut()[..len]).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Clear TX Fifo
    pub async fn clear_tx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_tx_fifo_cmd()).await
    }

    /// Return number of byte in TX FIFO
    pub async fn get_tx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_tx_fifo_level_req();
        let mut rsp = TxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Read data from the RX FIFO
    pub async fn rd_rx_fifo_to(&mut self, buffer: &mut[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data_rw(&[0,1], buffer).await
    }

    /// Read data from the RX FIFO to the local buffer
    pub async fn rd_rx_fifo(&mut self, len: usize) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&[0,1]).await?;
        self.spi
            .transfer_in_place(&mut self.buffer.data_mut()[..len]).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Return number of byte in RX FIFO
    pub async fn get_rx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rx_fifo_level_req();
        let mut rsp = RxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Clear RX FIFO
    pub async fn clear_rx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_rx_fifo_cmd()).await
    }

}