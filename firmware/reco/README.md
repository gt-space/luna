# RECO Board Driver

SPI driver for communication between the flight computer and the RECO (Recovery) board.

## Overview

The RECO board handles recovery mechanisms for the rocket and communicates with the flight computer via SPI. This driver provides a Rust interface for this communication.

## Hardware Setup

- **SPI Bus**: Typically `/dev/spidev0.0` or as configured
- **Chip Select**: GPIO controller 1, pin 16 (active low)
- **SPI Mode**: Mode 0 (CPOL=0, CPHA=0) - *adjust per RECO-FC Communication spec*
- **SPI Speed**: 1 MHz - *adjust per RECO-FC Communication spec*
- **Bits per Word**: 8

## Protocol Notes

⚠️ **IMPORTANT**: The current implementation uses placeholder command codes and protocol structure. The actual protocol should be updated based on the `RECO-FC_Communication_V1.pdf` specification document.

### Placeholder Protocol Structure

The driver currently uses placeholder command codes:
- `0x01` - Read status
- `0x02` - Read register
- `0x03` - Write register
- `0x04` - Enable channel
- `0x05` - Disable channel
- `0x06` - Heartbeat
- `0x07` - Reset

**These must be updated to match the actual RECO-FC Communication protocol.**

## Usage

### Basic Example

```rust
use reco::RecoDriver;
use common::comm::gpio::{Gpio, Pin, PinMode::Output, PinValue::High};

// Initialize GPIO controller
let gpio = Gpio::open_controller(1);
let mut cs_pin = gpio.get_pin(16);
cs_pin.mode(Output);
cs_pin.digital_write(High);

// Create driver
let mut reco = RecoDriver::new("/dev/spidev0.0", Some(cs_pin))
    .expect("Failed to initialize RECO driver");

// Read status
let status = reco.read_status().expect("Failed to read status");
println!("RECO status: {:?}", status);

// Enable recovery channel 1
reco.enable_channel(1).expect("Failed to enable channel");

// Check heartbeat
if reco.heartbeat().expect("Heartbeat failed") {
    println!("RECO board is responding");
}
```

### Reading Registers

```rust
// Read a specific register
let value = reco.read_register(0x10)
    .expect("Failed to read register");
println!("Register value: 0x{:02X}", value);
```

### Writing Registers

```rust
// Write to a register
reco.write_register(0x10, 0x42)
    .expect("Failed to write register");
```

### Command Execution

```rust
use reco::{RecoCommand, RecoDriver};

// Execute commands
reco.execute_command(RecoCommand::EnableChannel(1))?;
reco.execute_command(RecoCommand::ReadStatus)?;
```

## Testing

Example test scripts are available in the `examples/` directory.

### Running Examples

```bash
cd firmware/reco

# Basic communication test
cargo run --example basic_test

# Channel control test
cargo run --example channel_test

# Status monitoring
cargo run --example status_monitor
```

## Integration with Flight Computer

The flight computer (in `flight2`) can use this driver to:

1. Monitor RECO board status
2. Control recovery channels
3. Send commands and receive status updates
4. Handle recovery system operations

## Updating the Protocol

To update the driver with the actual protocol from the specification:

1. Review `RECO-FC_Communication_V1.pdf`
2. Update command codes in `src/lib.rs`
3. Update message structures to match the spec
4. Adjust SPI parameters (mode, speed, etc.) as needed
5. Update register addresses and command formats
6. Test with actual hardware

## Dependencies

- `spidev` - SPI communication
- `common` - GPIO pin control (with `gpio` feature)

## Error Handling

The driver provides comprehensive error types:

- `RecoError::SPI` - SPI communication errors
- `RecoError::InvalidChannel` - Invalid channel number
- `RecoError::InvalidRegister` - Invalid register address
- `RecoError::Protocol` - Protocol violations
- `RecoError::Timeout` - Operation timeouts
- `RecoError::DeviceNotResponding` - Device communication failures

## Notes

- Chip select is active low
- All SPI operations are synchronous
- The driver handles chip select automatically
- Register addresses and command formats need to be updated per the specification

