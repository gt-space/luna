# Flight computer (`ctv-flight-computer`)

Rust binary for the flight computer control loop. Crate package name: `ctv-flight-computer`.

## Running

From the **workspace root** (`luna/`):

```bash
cargo run -p ctv-flight-computer --release
```

From `luna/flight2/`:

```bash
cargo run --release
```

### Flight computer rev 4 SPI5 setup

Use the Pi image's two-chip-select SPI5 overlay, documented as `dtoverlay=spi5-2cs,cs0_pin=12,cs1_pin=26`.
Linux should expose `/dev/spidev5.0` for the ASM330LHBG1 and `/dev/spidev5.1` for the ADS124S06.
The kernel controls both chip selects. The ADC uses GPIO27 for data ready, and the IMU polls its status register.
The IMU publishes sensor-frame axes in m/s² and deg/s. Verify the installed sensor orientation before interpreting those axes as CTV body axes.

### Ethernet TF-Luna LiDARs

The flight computer connects to both TF-Luna adapters on startup. Use the normal
flight computer command from the workspace root.

```bash
cargo run -p ctv-flight-computer --release
```

| Sensor | TCP endpoint |
| --- | --- |
| LiDAR 1 | `192.168.1.200:2000` |
| LiDAR 2 | `192.168.1.201:2000` |

Both UART-to-Ethernet adapters use TCP Server mode, UART at 115200 baud with 8 data
bits, 1 stop bit, and no parity, subnet mask `255.255.255.0`, gateway `192.168.1.1`,
and DHCP off. Configure those settings on the adapters. The driver expects the TF-Luna
nine-byte centimeter output and publishes distances in millimeters.

Use `disable-lidar` to skip both adapters. `desktop` also skips the LiDAR workers.
For a bench run on the Pi with only the LiDAR hardware, pass the other sensor disable
commands together (`disable-gps-reco disable-imu disable-magnetometer disable-barometer`).

The FC saves LiDAR readings in its regular `flight_data_*.postcard` logs.
`postcard-to-csv` exports them under `sensor_readings.fc_lidar_*`.
After 350 ms without a complete checksum-valid frame, the FC marks the last sample stale.
Snapshots can repeat or skip sensor frames. Use the sample ID and FC-monotonic receive
timestamp to identify samples.

In the GUI, select **Umbilical** telemetry and open **AVI → FC Sensors** to view the
distance, signal strength, temperature, and status.

### Disable FC-local sensors

The flight computer accepts stackable command words to disable individual
workers and FC-local sensors without touching their hardware interfaces:

```bash
cargo run -p ctv-flight-computer --release -- disable-gps-reco
cargo run -p ctv-flight-computer --release -- disable-imu
cargo run -p ctv-flight-computer --release -- disable-magnetometer disable-barometer
cargo run -p ctv-flight-computer --release -- disable-lidar
```

These can be combined in any order. For example, this disables both the IMU and
magnetometer while leaving the barometer enabled:

```bash
cargo run -p ctv-flight-computer --release -- disable-imu disable-magnetometer
```

### Desktop mode (no GPS/RECO or FC-local sensor workers)

Use the `desktop` command to skip the GPS/RECO, MAG+BAR, IMU+ADC, and Ethernet LiDAR
workers. This is useful when running on a computer that is not an
embedded device (ie. MC laptops, meerkats, etc).

```bash
cargo run -p ctv-flight-computer --release -- desktop
```

### Build only, then run

`cargo build` does not accept program arguments like `desktop` as those are runtime commands. To use runtime commands while building, compile first:

```bash
cargo build -p ctv-flight-computer --release
```

Then run the release binary from the workspace root (artifact path is under `luna/target/release/`). A few examples of some subcommands that you can run the binary with are also listed:

```bash
./target/release/ctv-flight-computer
./target/release/ctv-flight-computer desktop
./target/release/ctv-flight-computer disable-gps-reco disable-imu
./target/release/ctv-flight-computer disable-imu disable-magnetometer
```

If you built from inside `flight2/`, the binary is still emitted at the workspace `target/` when this crate is part of the workspace (default layout: `../target/release/ctv-flight-computer` relative to `flight2/`).

## CLI flags

The binary also accepts global flags in addition to the runtime commands above:

- `--disable-file-logging` turns off on-disk vehicle-state logging.
- `--log-dirs <PATH>` writes logs to a custom directory instead of `$HOME/flight_logs`.
  - This flag is stackable. Repeat it to mirror logs to multiple directories.
- `--log-buffer-size <N>` changes the in-memory logging channel size. Default: `100`.
- `--log-rotation-mb <N>` rotates to a new log file once the current file reaches `N` MB. Default: `100`.
- `--fsync-rate <N>` forces buffered log data to sync to disk every `N` internal flushes. Default: `0` (disabled).
- `--print-gps` prints GPS data to the terminal at about 1 Hz.
- `--shadow-nav` runs the GMEKF navigation filter in shadow mode (logged only, never used for control). See below.
- `--shadow-nav-config <FILE>` reads JSON settings for it and implies `--shadow-nav`.

Flags can be used on their own or combined with runtime commands such as `desktop`
or `disable-imu`.

### File logging examples

By default, file logging is enabled and writes to `$HOME/flight_logs`.

```bash
cargo run -p ctv-flight-computer --release
./target/release/ctv-flight-computer
```

Write logs to a specific directory:

```bash
cargo run -p ctv-flight-computer --release -- --log-dirs /home/ubuntu/flight_logs
./target/release/ctv-flight-computer --log-dirs /home/ubuntu/flight_logs
```

Write the same logs to multiple directories, such as both blackbox SD card
mounts on the Raspberry Pi:

```bash
cargo run -p ctv-flight-computer --release -- --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
./target/release/ctv-flight-computer --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
```

Rotate files at 25 MB instead of the default 100 MB:

```bash
cargo run -p ctv-flight-computer --release -- --log-rotation-mb 25
./target/release/ctv-flight-computer --log-rotation-mb 25
```

Increase the log buffer for bursty disk I/O:

```bash
cargo run -p ctv-flight-computer --release -- --log-buffer-size 500
./target/release/ctv-flight-computer --log-buffer-size 500
```

Disable file logging entirely:

```bash
cargo run -p ctv-flight-computer --release -- --disable-file-logging
./target/release/ctv-flight-computer --disable-file-logging
```

Combine logging flags with runtime commands:

```bash
cargo run -p ctv-flight-computer --release -- --log-dirs /home/ubuntu/flight_logs disable-imu disable-magnetometer
./target/release/ctv-flight-computer --log-dirs /home/ubuntu/flight_logs disable-imu disable-magnetometer
```

### Shadow navigation (GMEKF)

`--shadow-nav` runs the GMEKF navigation filter on the IMU, barometer,
magnetometer and GPS
in its own thread and logs its estimate to `shadow_nav_<unix time>.csv` in the
first log directory. It never feeds control. Keep the vehicle still for the
first 2 s: the filter levels from the accelerometer, takes the pad pressure as
its barometer reference and the average gyro as the gyro bias. Heading comes
from the magnetometer (Earth field `mag_field_ned`, Atlanta by default). The
magnetometer is not calibrated for the vehicle's own field (motors, ESC
currents, frame) yet: if the startup message warns that it disagrees with the
Earth field, heading is not trustworthy; `"use_magnetometer": false` turns it
off.

```bash
./target/release/flight-computer --shadow-nav
./target/release/flight-computer --shadow-nav-config shadow_nav.json
```

`shadow_nav.json` is optional; every field has a default. The most likely ones
to set are the IMU mounting and the GPS antenna position:

```json
{
  "imu_to_body": ["+x", "+y", "+z"],
  "mag_to_body": ["+x", "+y", "+z"],
  "gps_lever_arm": [0.0, 0.0, 0.0],
  "use_gps_velocity": false,
  "gps_interval_s": 1.0,
  "level_seconds": 2.0
}
```

`gps_interval_s` fuses at most one GPS fix per second by default: standalone
GPS error wanders slowly, so fusing every 20 Hz fix would make the filter
overconfident.

`imu_to_body` says which IMU axis each body axis (x, y, z) points along, with
body x up on the pad. For example `["-z", "+y", "+x"]` means body x is the
IMU's -z axis, body y its +y and body z its +x. Read the IMU axes off the
datasheet diagram and how the board sits in the vehicle. A mapping that is not
a rotation (an axis used twice, or a mirror image) is rejected at startup and
shadow navigation stays off. A 3x3 matrix, row by row, also works.

While it runs it prints one status line per second (`status_every_s`; 0 turns
it off): position, velocity, tilt from vertical, heading, body rate, biases,
and whether the barometer, GPS and magnetometer readings were used (`ok`) or
rejected
(`GATED`). The full list of settings is `ShadowNavConfig` in
`src/shadow_nav.rs`.

#### Shadow navigation health

The filter publishes its latest estimate with a health flag for the control
loop: `ShadowNav::estimate(now)` returns the estimate only when it is
`Healthy`, otherwise why not (`Leveling`, `Unhealthy(reason)` or
`Failed(reason)`). It is unhealthy while any position, velocity, tilt or
heading uncertainty is above its limit (`max_position_std_m`,
`max_velocity_std_mps`, `max_tilt_std_deg`, `max_heading_std_deg`), while the
barometer or GPS has been rejected `max_rejections_in_a_row` times in a row,
or when no IMU step arrived for `imu_timeout_ms`. A non-finite estimate is
`Failed` until restart. The status line and the log's `health` column (0
leveling, 1 healthy, 2 unhealthy, 3 failed) show it.

Horizontal velocity needs GPS: without a fix the estimate turns unhealthy about
2 s after start. With a 1 Hz fix it is unhealthy from about 5 s to 15 s after
start and then healthy (velocity uncertainty settles near 0.12 m/s), so wait
about 20 s after startup, still on the pad with a GPS fix, before relying on
it.

### Raspberry Pi Blackbox Mounts

`scripts/mount_blackbox.sh` prepares two removable storage locations for file
logging on the Raspberry Pi. These locations are hardcoded, and correspond to
the column of USB ports that are closest to the ethernet port on a pi5.
The script is intended to run from a systemd service during boot, wait briefly
for block devices to appear, create the mount directories, and mount:

- `/dev/sda1` at `/mnt/blackbox_a`
- `/dev/sdb1` at `/mnt/blackbox_b`

Once those mount points are available, start the flight computer with both paths
listed under `--log-dirs` to mirror logs to both SD cards:

```bash
./target/release/ctv-flight-computer --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
```
