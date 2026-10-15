#![no_std]
#![no_main]

#[cfg(feature = "tx")]
use defmt::error;
use defmt::info;
#[cfg(not(feature = "tx"))]
use defmt::warn;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    peripherals, spim,
};
#[cfg(feature = "tx")]
use embassy_time::Timer;
use embassy_time::{Duration, with_timeout};
use lr2021::{
    Lr2021,
    lora::{LoraBw, LoraModulationParams, LoraPacketParams, Sf},
    radio::{PacketType, RampTime, RxBoost, RxPath},
    status::{
        IRQ_MASK_CRC_ERROR, IRQ_MASK_HEADER_ERR, IRQ_MASK_RX_DONE, IRQ_MASK_TIMEOUT,
        IRQ_MASK_TX_DONE, Intr,
    },
    system::{DioNum, TcxoVoltage},
};
#[cfg(not(feature = "tx"))]
use lr2021_test::decode_frame;
#[cfg(feature = "tx")]
use lr2021_test::encode_frame;
use lr2021_test::{FRAME_LEN, FREQUENCY_HZ};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SERIAL00 => spim::InterruptHandler<peripherals::SERIAL00>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // Semtech Wio-LR2021 shield on the Seeed XIAO nRF54L15.
    // D8/SCK=P2.01, D10/MOSI=P2.02, D9/MISO=P2.04.
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M8;
    let spi = spim::Spim::new(p.SERIAL00, Irqs, p.P2_01, p.P2_02, p.P2_04, spi_config);
    let reset = Output::new(p.P1_06, Level::High, OutputDrive::Standard); // D2
    let nss = Output::new(p.P1_07, Level::High, OutputDrive::Standard); // D3
    let busy = Input::new(p.P1_05, Pull::Up); // D1, open drain
    let mut irq = Input::new(p.P1_04, Pull::Down); // D0, radio DIO8
    let mut radio = Lr2021::new_blocking(reset, busy, spi, nss);

    radio.reset().await.expect("LR2021 reset");
    let version = radio.get_version().await.expect("LR2021 SPI/version");
    info!("LR2021 firmware: {}", version);
    radio
        .set_regulator_mode(true)
        .await
        .expect("DCDC regulator");
    radio
        .set_tcxo(TcxoVoltage::Tcxo1v8, 0)
        .await
        .expect("1.8 V TCXO");
    radio
        .set_rf(FREQUENCY_HZ)
        .await
        .expect("2.44 GHz frequency");
    radio
        .set_rx_path(RxPath::HfPath, RxBoost::Off)
        .await
        .expect("HF RX path");
    radio.set_pa_hf().await.expect("HF PA");
    radio.calib_fe(&[]).await.expect("front-end calibration");

    let modulation = LoraModulationParams::basic(Sf::Sf7, LoraBw::Bw500);
    let packet = LoraPacketParams::basic(FRAME_LEN as u8, &modulation);
    radio
        .set_packet_type(PacketType::Lora)
        .await
        .expect("LoRa packet type");
    radio
        .set_lora_modulation(&modulation)
        .await
        .expect("LoRa modulation");
    radio
        .set_lora_packet(&packet)
        .await
        .expect("LoRa packet format");
    radio.set_lora_syncword(0x12).await.expect("test sync word");
    radio
        .set_tx_params(0, RampTime::Ramp8u)
        .await
        .expect("0 dBm TX power");
    radio
        .set_dio_irq(
            DioNum::Dio8,
            Intr::new(
                IRQ_MASK_RX_DONE
                    | IRQ_MASK_TX_DONE
                    | IRQ_MASK_CRC_ERROR
                    | IRQ_MASK_HEADER_ERR
                    | IRQ_MASK_TIMEOUT,
            ),
        )
        .await
        .expect("DIO8 IRQ");
    let _ = radio.get_and_clear_irq().await.expect("clear initial IRQs");

    #[cfg(feature = "tx")]
    run_tx(&mut radio, &mut irq).await;
    #[cfg(not(feature = "tx"))]
    run_rx(&mut radio, &mut irq).await;
}

#[cfg(feature = "tx")]
async fn run_tx<O, S, B>(radio: &mut Lr2021<O, S, B>, irq: &mut Input<'_>) -> !
where
    O: embedded_hal::digital::OutputPin,
    S: embedded_hal_async::spi::SpiBus<u8>,
    B: lr2021::BusyPin,
{
    info!("TX: 2440 MHz LoRa SF7/BW500, 0 dBm, one frame per second");
    let mut sequence = 0u32;
    loop {
        let frame = encode_frame(sequence);
        radio.wr_tx_fifo_from(&frame).await.expect("write TX FIFO");
        radio.set_tx(0).await.expect("start TX");
        match with_timeout(Duration::from_secs(2), irq.wait_for_high()).await {
            Ok(()) => {
                let flags = radio.get_and_clear_irq().await.expect("TX IRQ status");
                if flags.intr_match(IRQ_MASK_TX_DONE) {
                    info!("TX seq={}", sequence);
                    sequence = sequence.wrapping_add(1);
                } else {
                    error!("TX failed, IRQ flags=0x{:08x}", flags.value());
                }
            }
            _ => {
                error!("TX IRQ timeout");
                let _ = radio.get_and_clear_irq().await;
            }
        }
        Timer::after_secs(1).await;
    }
}

#[cfg(not(feature = "tx"))]
async fn run_rx<O, S, B>(radio: &mut Lr2021<O, S, B>, irq: &mut Input<'_>) -> !
where
    O: embedded_hal::digital::OutputPin,
    S: embedded_hal_async::spi::SpiBus<u8>,
    B: lr2021::BusyPin,
{
    info!("RX: 2440 MHz LoRa SF7/BW500, waiting for test frames");
    radio.set_rx_continous().await.expect("continuous RX");
    let mut received = 0u32;
    let mut expected = None::<u32>;
    loop {
        match with_timeout(Duration::from_secs(5), irq.wait_for_high()).await {
            Ok(()) => {}
            Err(_) => {
                info!("RX idle; valid frames={}", received);
                continue;
            }
        }
        let flags = radio.get_and_clear_irq().await.expect("RX IRQ status");
        if flags.intr_match(IRQ_MASK_CRC_ERROR | IRQ_MASK_HEADER_ERR) {
            warn!("RX CRC/header error, IRQ flags=0x{:08x}", flags.value());
            continue;
        }
        if !flags.intr_match(IRQ_MASK_RX_DONE) {
            warn!("RX IRQ flags=0x{:08x}", flags.value());
            continue;
        }
        let len = radio.get_rx_pkt_len().await.expect("RX packet length") as usize;
        if len != FRAME_LEN {
            warn!("RX unexpected length={}", len);
            // Drain the packet so the FIFO stays in sync for the next one.
            if len <= 255 {
                let mut discard = [0u8; 255];
                radio
                    .rd_rx_fifo_to(&mut discard[..len])
                    .await
                    .expect("drain RX FIFO");
            }
            continue;
        }
        let mut frame = [0u8; FRAME_LEN];
        radio.rd_rx_fifo_to(&mut frame).await.expect("read RX FIFO");
        let status = radio
            .get_lora_packet_status()
            .await
            .expect("LoRa packet status");
        match decode_frame(&frame) {
            Ok(sequence) => {
                received = received.wrapping_add(1);
                if let Some(next) = expected {
                    if sequence != next {
                        warn!("RX gap: expected seq={}, got={}", next, sequence);
                    }
                }
                expected = Some(sequence.wrapping_add(1));
                info!(
                    "RX seq={} count={} RSSI=-{}.{} dBm SNR(qdB)={}",
                    sequence,
                    received,
                    status.rssi_pkt() / 2,
                    (status.rssi_pkt() % 2) * 5,
                    status.snr_pkt()
                );
            }
            Err(_) => warn!("RX invalid test frame"),
        }
    }
}
