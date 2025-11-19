# GPS Driver for u-blox ZED-F9P

A Rust driver for the u-blox ZED-F9P high-precision GNSS module using I2C communication on Raspberry Pi.

## Features

- ✅ I2C communication with u-blox ZED-F9P module
- ✅ Query module version information (MON-VER)
- ✅ Poll for Position, Velocity, and Time (NAV-PVT)
- ✅ Read PVT data in periodic mode (no polling required)
- ✅ Velocity reported in North-East-Down (NED) coordinate system
- ✅ Configure measurement rate (up to 25 Hz, tested at 20 Hz)
- ✅ Configure message rates
- ✅ Full UBX protocol support via `ublox` crate (v0.7)

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

### Basic Example (Polling Mode)

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
        
        if let Some(vel) = pvt.velocity {
            println!("Velocity (NED): north={:.2} m/s, east={:.2} m/s, down={:.2} m/s",
                vel.north, vel.east, vel.down);
        }
    }
    
    Ok(())
}
```

### Periodic Mode Example (20 Hz)

```rust
use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut gps = GPS::new(1, None)?;
    
    // Configure measurement rate to 20 Hz (50 ms period)
    gps.set_measurement_rate(50, 1, 0)?;
    
    // Configure NAV-PVT to send on every solution
    gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0])?;
    
    // Read PVT data as it arrives (no polling needed)
    loop {
        if let Some(pvt) = gps.read_pvt()? {
            if let Some(pos) = pvt.position {
                println!("Position: lat={:.7}°, lon={:.7}°, alt={:.2}m",
                    pos.lat, pos.lon, pos.alt);
            }
        }
        thread::sleep(Duration::from_millis(10));
    }
}
```

### Run the Examples

```bash
# Basic polling mode
cargo run --example basic_usage

# Periodic mode at 20 Hz
cargo run --example periodic_mode

# Continuous data reading
cargo run --example continuous_read
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

The `PVT` structure contains:
- `position`: Optional `Position` with latitude, longitude, and altitude
- `velocity`: Optional `NedVelocity` with north, east, and down components (in m/s)
- `time`: Optional `DateTime<Utc>` timestamp

### `NedVelocity`

Velocity in the North-East-Down (NED) coordinate system:
- `north`: North component of velocity (m/s)
- `east`: East component of velocity (m/s)
- `down`: Down component of velocity (m/s)

All velocity values are in meters per second. The NED coordinate system is commonly used in aerospace applications where:
- **North** is positive toward the North Pole
- **East** is positive toward 90° longitude
- **Down** is positive toward the center of the Earth

### `GPS::set_measurement_rate(&mut self, meas_rate_ms: u16, nav_rate: u16, time_ref: u16) -> Result<(), GPSError>`

Configures the measurement rate (CFG-RATE) of the GPS module. This determines how often the module calculates navigation solutions.

- `meas_rate_ms`: Measurement period in milliseconds (e.g., 50 for 20 Hz, 100 for 10 Hz)
- `nav_rate`: Navigation rate (number of measurement cycles per navigation solution, typically 1)
- `time_ref`: Time reference (0 = UTC, 1 = GPS time)

**Example for 20 Hz operation:**
```rust
gps.set_measurement_rate(50, 1, 0)?;  // 50 ms = 20 Hz, nav_rate=1, UTC time
```

The ZED-F9P supports measurement rates up to 25 Hz. For 20 Hz operation, set `meas_rate_ms` to 50.

### `GPS::set_nav_pvt_rate(&mut self, rate: [u8; 6]) -> Result<(), GPSError>`

Configures the rate at which NAV-PVT messages are sent. The rate array corresponds to:
`[I2C/DDC, UART1, UART2, USB, SPI, Reserved]`

Example: `[1, 0, 0, 0, 0, 0]` sends NAV-PVT on every navigation solution to I2C.

**Note:** This only configures message output rate. You must also configure the measurement rate using `set_measurement_rate()` to achieve high-frequency updates.

### `GPS::read_pvt(&mut self) -> Result<Option<PVT>, GPSError>`

Reads available packets and extracts PVT data if found. Unlike `poll_pvt()`, this function does not send a poll request - it simply reads any available NAV-PVT packets from the module's buffer.

This is useful in periodic mode where the module automatically sends NAV-PVT messages at a configured rate.

**Returns:**
- `Ok(Some(PVT))` - If a NAV-PVT packet was found and parsed
- `Ok(None)` - If no NAV-PVT packet was found in the available data
- `Err(GPSError)` - If an I2C error occurred

**Example:**
```rust
// In periodic mode, read PVT data as it arrives
if let Some(pvt) = gps.read_pvt()? {
    // Process PVT data
}
```

## I2C Protocol Notes

The driver implements the u-blox I2C (DDC) protocol:

1. **Reading data:**
   - Read 2 bytes from registers `0xFD/0xFE` to get the number of available bytes (high byte at `0xFD`, low byte at `0xFE`).
   - Read **exactly that many bytes** from the data stream register `0xFF`.
   - Parse using the UBX protocol parser.
   - The implementation avoids reading a fixed 1240‑byte buffer of `0xFF` when little or no data is available, which keeps the I²C transaction time small and predictable.

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
│   ├── basic_usage.rs      # Basic polling mode example
│   ├── periodic_mode.rs    # Periodic mode example (20 Hz)
│   ├── continuous_read.rs  # Continuous data reading example
│   ├── satellite_info.rs   # Satellite information example
│   ├── satellite_monitor.rs # Satellite signal monitoring
│   └── ...                 # Additional diagnostic examples
├── Cargo.toml              # Rust dependencies
├── README.md               # This file (main documentation)
```

## High-Frequency Operation (20 Hz)

The driver supports high-frequency GPS updates up to 25 Hz. For 20 Hz operation:

1. **Configure measurement rate:**
   ```rust
   gps.set_measurement_rate(50, 1, 0)?;  // 50 ms = 20 Hz
   ```

2. **Configure message rate:**
   ```rust
   gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0])?;  // Send on every solution
   ```

3. **Read data in a loop:**
   ```rust
   loop {
       if let Some(pvt) = gps.read_pvt()? {
           // Process PVT data at 20 Hz
       }
       thread::sleep(Duration::from_millis(10));
   }
   ```

**I2C Throughput:** At 400 kHz I2C speed, 20 Hz operation is easily supported. Each NAV-PVT message is approximately 100 bytes, resulting in ~2 KB/s data rate, well within I2C capacity.

See `examples/periodic_mode.rs` for a complete 20 Hz example.

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
