# RECO Board Driver

SPI driver for communication between the flight computer and the RECO (Recovery) board.

## Overview

The RECO board handles recovery mechanisms for the rocket and communicates with the flight computer via SPI using a custom protocol. This driver provides a Rust interface for this communication, designed for Raspberry Pi hardware.

## Hardware Setup

- **Platform**: Raspberry Pi
- **SPI Bus**: SPI0 (or SPI1)
- **SPI Mode**: Mode 0 (CPOL=0, CPHA=0)
- **SPI Speed**: 2 MHz
- **Chip Select**: Hardware CS (CE0/CE1) - automatically controlled by kernel driver
- **SPI Library**: Linux spidev (via ioctl)

### Enabling SPI on Raspberry Pi

**For SPI0 (default, usually already enabled):**
```bash
# Check if SPI0 is enabled
ls /dev/spi*

# If not enabled, add to /boot/config.txt (or /boot/firmware/config.txt on newer OS):
dtparam=spi=on
```

**For SPI1 (must be explicitly enabled):**
```bash
# Add to /boot/config.txt (or /boot/firmware/config.txt on newer OS):
dtoverlay=spi1-1cs

# Or for SPI1 with 3 CS lines:
dtoverlay=spi1-3cs

# After adding, reboot:
sudo reboot

# Verify SPI1 is enabled:
ls /dev/spi*
# Should show: /dev/spidev0.0, /dev/spidev0.1, /dev/spidev1.0, /dev/spidev1.1
```

**Note:** On newer Raspberry Pi OS versions, the config file may be at `/boot/firmware/config.txt` instead of `/boot/config.txt`.

## Message Protocol

### Messages TO RECO (from FC)

All messages sent to RECO follow this format:
- **Opcode** (1 byte)
- **Body** (25 bytes of data)
- **Total**: 26 bytes

All SPI commands execute as 144-byte full-duplex transfers so that RECO telemetry is clocked out on every exchange. Commands that do not require the telemetry simply discard the received bytes.

#### Opcode 0x01: Launched

Indicates that the rocket has been launched.

```rust
reco.send_launched()?;
```

- Opcode: `0x01`
- Body: All zeros (25 bytes padding)
- Transfer length: 144 bytes (full-duplex); telemetry received during this command is ignored

#### Opcode 0x02: GPS Data / Telemetry Exchange

Sends GPS data to RECO while simultaneously reading the latest telemetry frame.

```rust
use reco::FcGpsBody;

let gps_data = FcGpsBody {
    velocity_north: 10.5,
    velocity_east: 2.3,
    velocity_down: -5.1,
    latitude: 37.7749,
    longitude: -122.4194,
    altitude: 100.0,
    valid: true,
};

let reco_data = reco.send_gps_data_and_receive_reco(&gps_data)?;
println!("RECO quaternion: {:?}", reco_data.quaternion);
```

- Opcode: `0x02`
- Body structure:
  - `velocity_north` (f32, 4 bytes)
  - `velocity_east` (f32, 4 bytes)
  - `velocity_down` (f32, 4 bytes)
  - `latitude` (f32, 4 bytes)
  - `longitude` (f32, 4 bytes)
  - `altitude` (f32, 4 bytes)
  - `valid` (bool, 1 byte)
- Transfer length: 144 bytes. The outbound GPS payload occupies the first 26 bytes of the transfer, and the driver returns the 144-byte RECO telemetry frame (`RecoBody`) gathered during the exchange.

#### Opcode 0x03: Voting Logic

Configures voting logic for the three processors on RECO.

```rust
use reco::VotingLogic;

let voting_logic = VotingLogic {
    processor_1_enabled: true,
    processor_2_enabled: true,
    processor_3_enabled: false,
};

reco.send_voting_logic(&voting_logic)?;
```

- Opcode: `0x03`
- Body structure:
  - `processor_1_enabled` (bool, 1 byte)
  - `processor_2_enabled` (bool, 1 byte)
  - `processor_3_enabled` (bool, 1 byte)
  - Padding (22 bytes, all zeros)
- Transfer length: 144 bytes; RECO telemetry shifted in during the transfer is currently discarded by the driver

### Messages FROM RECO (to FC)

RECO sends a single message type containing sensor and state data:

```rust
let data = reco.receive_data()?;
```

- **Body** (144 bytes): `RecoBody` structure
- **Total**: 144 bytes

The `RecoBody` structure contains:
- `quaternion[4]` (f32 × 4 = 16 bytes) - Vehicle attitude
- `lla_pos[3]` (f32 × 3 = 12 bytes) - Position [longitude, latitude, altitude]
- `velocity[3]` (f32 × 3 = 12 bytes) - Velocity
- `g_bias[3]` (f32 × 3 = 12 bytes) - Gyroscope bias offset
- `a_bias[3]` (f32 × 3 = 12 bytes) - Accelerometer bias offset
- `g_sf[3]` (f32 × 3 = 12 bytes) - Gyro scale factor
- `a_sf[3]` (f32 × 3 = 12 bytes) - Acceleration scale factor
- `lin_accel[3]` (f32 × 3 = 12 bytes) - XYZ Acceleration
- `angular_rate[3]` (f32 × 3 = 12 bytes) - Angular Rates (pitch, yaw, roll)
- `mag_data[3]` (f32 × 3 = 12 bytes) - XYZ Magnetometer Data
- `temperature` (f32, 4 bytes)
- `pressure` (f32, 4 bytes)
- `stage1_enabled` (bool, 1 byte) - Stage 1 recovery enabled status
- `stage2_enabled` (bool, 1 byte) - Stage 2 recovery enabled status
- `vref_a_stage1`, `vref_a_stage2` (bool, 1 byte each) - Voltage reference A status
- `vref_b_stage1`, `vref_b_stage2` (bool, 1 byte each) - Voltage reference B status
- `vref_c_stage1`, `vref_c_stage2` (bool, 1 byte each) - Voltage reference C status
- `vref_d_stage1`, `vref_d_stage2` (bool, 1 byte each) - Voltage reference D status
- `vref_e_stage1_1`, `vref_e_stage1_2` (bool, 1 byte each) - Voltage reference stage 1 status

## Usage

### Basic Example

```rust
use reco::{RecoDriver, FcGpsBody};

// Initialize driver with hardware CS (CE1 on SPI1)
let mut reco = RecoDriver::new("/dev/spidev1.1")?;

// Send "launched" message
reco.send_launched()?;

// Send GPS data and capture RECO telemetry in the same transfer
let gps_data = FcGpsBody {
    velocity_north: 10.5,
    velocity_east: 2.3,
    velocity_down: -5.1,
    latitude: 37.7749,
    longitude: -122.4194,
    altitude: 100.0,
    valid: true,
};
let reco_snapshot = reco.send_gps_data_and_receive_reco(&gps_data)?;
println!("RECO temperature: {}°C", reco_snapshot.temperature);

// Request EKF initialization
reco.send_init_ekf()?;

// Receive data from RECO
let data = reco.receive_data()?;
println!("Temperature: {}°C", data.temperature);
println!("Pressure: {} Pa", data.pressure);
```

### Working with GPS Data

```rust
use reco::FcGpsBody;

let gps_data = FcGpsBody {
    velocity_north: 10.5,
    velocity_east: 2.3,
    velocity_down: -5.1,
    latitude: 37.7749,
    longitude: -122.4194,
    altitude: 100.0,
    valid: true,
};

let reco_snapshot = reco.send_gps_data_and_receive_reco(&gps_data)?;
println!("RECO altitude: {}", reco_snapshot.lla_pos[2]);
```

### Requesting EKF Initialization

```rust
// Ask RECO to initialize (or reinitialize) its EKF
reco.send_init_ekf()?;
```

### Receiving Data

```rust
// Receive data from RECO
match reco.receive_data() {
    Ok(data) => {
        println!("Quaternion: {:?}", data.quaternion);
        println!("Position: {:?}", data.lla_pos);
        println!("Velocity: {:?}", data.velocity);
        println!("Temperature: {}°C", data.temperature);
        println!("Pressure: {} Pa", data.pressure);
        println!("Stage 1 enabled: {}", data.stage1_enabled);
        println!("Stage 2 enabled: {}", data.stage2_enabled);
        println!("Vref A: [{}, {}]", data.vref_a_stage1, data.vref_a_stage2);
    }
    Err(e) => {
        eprintln!("Failed to receive data: {}", e);
        // Handle error (SPI error, etc.)
    }
}
```

## Testing

Example scripts are available in the `examples/` directory.

### Running Examples

```bash
cd firmware/reco

# Basic communication test
cargo run --example basic_test

# Message protocol test
cargo run --example channel_test

# Data monitoring
cargo run --example status_monitor
```

## Integration with Flight Computer

The flight computer can use this driver to:

1. Send launch notification to RECO
2. Send GPS data for position/velocity updates while capturing RECO telemetry
3. Configure processor voting logic
4. Receive sensor and state data from RECO
5. Monitor RECO board status via received data

## Dependencies

- `libc` - System calls for spidev ioctl operations

## Error Handling

The driver provides comprehensive error types:

- `RecoError::Protocol` - Protocol violations and SPI communication errors
- `RecoError::InvalidMessageSize` - Invalid message size received
- `RecoError::Deserialization` - Data deserialization errors

## Notes

- **Chip select is active low** - Hardware CS (CE0/CE1) is automatically controlled by the kernel driver
- All SPI operations are synchronous
- The driver uses Linux spidev for SPI communication
- All f32 values are serialized in little-endian format
- Chip select is automatically asserted before each transfer and deasserted after completion
