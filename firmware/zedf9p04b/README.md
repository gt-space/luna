# GPS Driver for u-blox ZED-F9P

A Rust driver for the u-blox ZED-F9P high-precision GNSS module using I2C communication on Raspberry Pi.

## Features

- ✅ I2C communication with u-blox ZED-F9P module
- ✅ Query module version information (MON-VER)
- ✅ Poll for Position, Velocity, and Time (NAV-PVT)
- ✅ Configure message rates
- ✅ Full UBX protocol support via `ublox` crate

## Hardware Requirements

- Raspberry Pi (tested on Pi 5)
- u-blox ZED-F9P GNSS module
- I2C connection between Pi and module

## Important Note

⚠️ **This code must be compiled and run on a Raspberry Pi running Linux.** The `rppal` crate uses Linux-specific system calls and hardware interfaces that are not available on Windows or macOS.

If you're developing on Windows/macOS, you'll need to:
1. Cross-compile for ARM Linux, OR
2. Transfer the code to your Raspberry Pi and compile there

## Setup

### 1. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
```

Verify I2C is enabled:

```bash
ls /dev/i2c-*
# Should show: /dev/i2c-1
```

Check if the module is detected:

```bash
sudo i2cdetect -y 1
# Should show device at address 0x42
```

### 2. Add to your project

Add this to your `Cargo.toml`:

```toml
[dependencies]
zedf9p04b = { path = "../firmware/zedf9p04b" }  # or use git/crates.io once published
```

## Usage

### Basic Example

```rust
use zedf9p04b::GPS;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize GPS on I2C bus 1 with default address (0x42)
    let mut gps = GPS::new(1, None)?;
    
    // Query module version
    gps.mon_ver()?;
    
    // Poll for position data
    if let Some(pvt) = gps.poll_pvt()? {
        if let Some(pos) = pvt.position {
            println!("Lat: {}, Lon: {}, Alt: {}m", 
                pos.lat, pos.lon, pos.alt);
        }
    }
    
    Ok(())
}
```

### Run the Example

```bash
cargo run --example basic_usage
```

## API Documentation

### `GPS::new(i2c_bus: u8, address: Option<u16>) -> Result<GPS, GPSError>`

Creates a new GPS driver instance.

- `i2c_bus`: I2C bus number (typically 1 for `/dev/i2c-1`)
- `address`: Optional I2C address (defaults to `0x42`)

### `GPS::mon_ver(&mut self) -> Result<(), GPSError>`

Queries the module for version information. Useful for testing connectivity.

### `GPS::poll_pvt(&mut self) -> Result<Option<PVT>, GPSError>`

Polls for Position, Velocity, and Time data. Returns `None` if no GPS fix is available.

### `GPS::set_nav_pvt_rate(&mut self, rate: [u8; 6]) -> Result<(), GPSError>`

Configures the rate at which NAV-PVT messages are sent. The rate array corresponds to:
`[I2C/DDC, UART1, UART2, USB, SPI, Reserved]`

Example: `[1, 0, 0, 0, 0, 0]` sends NAV-PVT on every navigation solution to I2C.

## I2C Protocol Notes

The driver implements the u-blox I2C (DDC) protocol:

1. **Reading data:**
   - Read 2 bytes to get the number of available bytes
   - Read that many bytes from the data stream
   - Parse using the UBX protocol parser

2. **Writing data:**
   - Simply write UBX packets directly to the module

3. **Timing:**
   - The module may need 50-100ms to process requests
   - The driver includes appropriate delays for reliability

## Project Structure

```
firmware/zedf9p04b/
├── src/
│   └── lib.rs              # Main GPS driver implementation
├── examples/
│   ├── basic_usage.rs      # Basic usage example
│   └── continuous_read.rs  # Continuous data reading example
├── Cargo.toml              # Rust dependencies
├── README.md               # This file (main documentation)
```

## Troubleshooting

### "No such device" error

- Ensure I2C is enabled: `sudo raspi-config`
- Check device is connected: `sudo i2cdetect -y 1`
- Verify wiring connections

### "No response from device"

- Check power supply (3.3V or 5V depending on module)
- Ensure antenna is connected (module needs satellite signals)
- Try increasing delays in the code
- Verify I2C address is correct (default: 0x42)

### No GPS fix

- Ensure the module has a clear view of the sky
- Wait 30-60 seconds for cold start acquisition
- Check antenna connection
- The module needs to receive signals from at least 4 satellites for a 3D fix

## References

- [ZED-F9P Datasheet](https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf)
- [u-blox F9 Interface Description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)
- [ublox Rust crate](https://docs.rs/ublox/latest/ublox/)
- [RPPAL documentation](https://docs.rs/rppal/latest/rppal/)
