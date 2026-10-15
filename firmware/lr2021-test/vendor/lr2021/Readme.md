# LR2021 Driver

[![Crates.io](https://img.shields.io/crates/v/lr2021.svg)](https://crates.io/crates/lr2021)
[![Documentation](https://docs.rs/lr2021/badge.svg)](https://docs.rs/lr2021)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/TheClams/lr2021)

An async, no_std Rust driver for the Semtech LR2021 dual-band transceiver, supporting many different radio protocols including LoRa, BLE, ZigBee, Z-Wave, and more.

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
lr2021 = "0.14"
embassy-time = "0.5"
```

Basic usage:

```rust
use lr2021_driver::Lr2021;

let mut radio = Lr2021::new(reset_pin, busy_pin, spi_device, nss_pin);
radio.reset().await?;
// Configure and use your preferred protocol
```

## Hardware Requirements

- Semtech LR20xx transceiver module
- SPI-capable microcontroller
- 3 GPIO pins: Reset (output), Busy (input), NSS/CS (output) (not counting SPI SCK/MISO/MOSI)
- Embassy-compatible async runtime

## Documentation & Examples

- **[API Documentation](https://docs.rs/lr2021-driver)** - Complete API reference
- **[Example Applications](https://github.com/TheClams/lr2021-apps)** - Real-world usage examples on Nucleo boards

## Protocol Test Status

| Protocol | Status | Notes |
|----------|--------|-------|
| LoRa |**Partial** | Basic communication between two LR2021 devices: smallest SF, highest bandwidth. TODO: Ranging |
| BLE | **Partial** | 1MB/s mode, compatible with other BLE devices. TODO: 2Mb/s, Coded |
| FLRC | **Tested** | Basic communication between two LR2021 devices |
| FSK | **Tested** | Generic FSK communication verified |
| Z-Wave | **Tested** | Scan mode tested with ZStick S2, R1-R3 reception |
| OOK | **Partial** | ADSB reception validated, RTS between two LR2021 |
| ZigBee | **Partial** | Reception validated with standard device |
| WiSUN | **Partial** | Basic communication between two LR2021 devices |
| WMBus | **Partial** | Basic communication between two LR2021 devices |
| LR-FHSS | **Unplanned** | TX only (require gateway for test) |
| Sigfox (BPSK) | **Unplanned** | TX only (require gateway for test) |

# LR20xx family
The driver supports the whole LR20xx chip family: LR2012/LR2021/LR2022.
The only difference between each series is chip is the features supported:
 - LR2021 supports all possible features (enabled by default)
 - LR2022 does not support advanced FSK modulation such as FLRC/Zigbee/Zwave
 - LR2012 does not support advanced modulation nor 2.4GHz path

Features in the driver allows to make sure at compile time you are not using supported commands.

## LR2012
When targeting the LR2012 simply disable the default feature:
```toml
[dependencies]
lr2021 = {version = "0.14", default-features = false}
```

## LR2022
When targeting the LR2022, disable the default feature and enable the RF 2.4GHz path:
```toml
[dependencies]
lr2021 = {version = "0.14", default-features = false}

[features]
default = ["lr2021/rf2g4"]
```

