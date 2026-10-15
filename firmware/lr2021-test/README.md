# LR2021 2.4 GHz link test

Standalone Rust firmware for the two **Seeed XIAO nRF54L15 + Wio-LR2021 LoRa Plus EVK** boards. This crate is intentionally outside `flight2`. The frame codec in `src/lib.rs` has no board dependencies, so it can be reused by a future flight computer or telemetry process.

The default build is a receiver. Build the second board with `--features tx` to send one numbered 16-byte LoRa frame per second. Both use 2440 MHz, SF7, 500 kHz bandwidth, coding rate 4/5, explicit header, hardware CRC, sync word `0x12`, and 0 dBm transmit power. These are raw point-to-point LoRa packets, not LoRaWAN.

## Hardware

Fit the **2.4 GHz antenna** to each board's 2.4 GHz SMA port before transmitting. The firmware uses Semtech's EVK shield wiring:

| LR2021 signal | XIAO nRF54L15 | XIAO label |
| --- | --- | --- |
| SPI SCK | P2.01 | D8 |
| SPI MOSI | P2.02 | D10 |
| SPI MISO | P2.04 | D9 |
| NSS | P1.07 | D3 |
| RESET | P1.06 | D2 |
| BUSY | P1.05 | D1 |
| DIO8 IRQ | P1.04 | D0 |

The driver configures the EVK's 1.8 V TCXO, DC/DC regulator, HF receive path, and HF power amplifier. SPI runs in mode 0 at 8 MHz.

## Build and run

Install Rust 1.98, the `thumbv8m.main-none-eabihf` target, and `probe-rs`. From this directory:

```powershell
rustup target add thumbv8m.main-none-eabihf --toolchain 1.98.0
cargo build --release                 # receiver
Copy-Item target/thumbv8m.main-none-eabihf/release/lr2021-test target/thumbv8m.main-none-eabihf/release/lr2021-rx.elf
cargo build --release --features tx   # transmitter
Copy-Item target/thumbv8m.main-none-eabihf/release/lr2021-test target/thumbv8m.main-none-eabihf/release/lr2021-tx.elf
```

The two `.elf` files keep the receiver and transmitter builds separate. `probe-rs run` flashes and shows `defmt` logs through the XIAO's onboard debug probe. Use two terminals, one for each board. If both are plugged into the same computer, run `probe-rs list` and use each board's `VID:PID:Serial` shown there:

```powershell
# Receiver terminal
probe-rs run --chip nrf54l15 --probe VID:PID:RX_SERIAL target/thumbv8m.main-none-eabihf/release/lr2021-rx.elf
```

```powershell
# Transmitter terminal
probe-rs run --chip nrf54l15 --probe VID:PID:TX_SERIAL target/thumbv8m.main-none-eabihf/release/lr2021-tx.elf
```

With one board plugged in at a time, omit the `--probe` argument, flash the receiver, then flash the transmitter. The receiver should log `RX seq=...` with RSSI and SNR. A missing sequence produces `RX gap`, and CRC/header errors are logged separately. The transmitter logs `TX seq=...` after the radio reports TX done. Keep the boards a short distance apart for the first test.

To check the frame codec on a desktop host:

```powershell
cargo test --lib --target x86_64-pc-windows-msvc
```

## Driver and source references

- [`lr2021` 0.14.0](https://github.com/TheClams/lr2021) is the existing async, `no_std` Rust radio driver used here. Its [LoRa demonstration](https://github.com/TheClams/lr2021-apps/blob/master/src/bin/lora_txrx.rs) provided the command sequence.
- The [Semtech shield definition](https://github.com/Lora-net/usp_zephyr/blob/main/boards/shields/semtech_wio_lr20xx/semtech_wio_lr20xx_common.dtsi) provides NSS, RESET, BUSY, DIO8, and 8 MHz SPI settings; the [Seeed EVK guide](https://wiki.seeedstudio.com/semtech_lr2021_evk_getting_started/) provides the XIAO pin mapping.
- [`embassy-nrf` nRF54L15 example](https://github.com/embassy-rs/embassy/tree/main/examples/nrf54l15-app) provides the target, linker memory layout, and SPI initialization pattern.

The MIT-licensed `lr2021` 0.14.0 source is included in `vendor/lr2021`. Its only local change is removing the hardcoded `tick-hz-32_768` feature from its `embassy-time` dependency, because `embassy-nrf`'s nRF54L15 GRTC time driver uses a 1 MHz tick. No LR2021 driver Rust source was changed.

## Status

Both firmware roles link for `thumbv8m.main-none-eabihf`, and the host frame test passes. This machine had no debug probes attached (`probe-rs list` found none), so RF behavior and flashing still need validation on the two EVK boards.
