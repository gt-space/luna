# Flight computer (`flight-computer`)

Rust binary for the flight computer control loop. Crate package name: `flight-computer`.

## Running

From the **workspace root** (`luna/`):

```bash
cargo run -p flight-computer --release
```

From `luna/flight2/`:

```bash
cargo run --release
```

### Disable FC-local sensors

The flight computer accepts stackable command words to disable individual
workers and FC-local sensors without touching their hardware interfaces:

```bash
cargo run -p flight-computer --release -- disable-gps-reco
cargo run -p flight-computer --release -- disable-imu
cargo run -p flight-computer --release -- disable-magnetometer disable-barometer
```

These can be combined in any order. For example, this disables both the IMU and
magnetometer while leaving the barometer enabled:

```bash
cargo run -p flight-computer --release -- disable-imu disable-magnetometer
```

### Desktop mode (no GPS/RECO or FC-local sensor workers)

Use the `desktop` command to skip starting the GPS/RECO worker and the
MAG+BAR and IMU+ADC workers. This is useful when running on a computer that is not an
embedded device (ie. MC laptops, meerkats, etc).

```bash
cargo run -p flight-computer --release -- desktop
```

### Build only, then run

`cargo build` does not accept program arguments like `desktop`as those are runtime commands. To use runtime commands while building, compile first:

```bash
cargo build -p flight-computer --release
```

Then run the release binary from the workspace root (artifact path is under `luna/target/release/`). A few examples of some subcommands that you can run the binary with are also listed:

```bash
./target/release/flight-computer
./target/release/flight-computer desktop
./target/release/flight-computer disable-gps-reco disable-imu
./target/release/flight-computer disable-imu disable-magnetometer
```

If you built from inside `flight2/`, the binary is still emitted at the workspace `target/` when this crate is part of the workspace (default layout: `../target/release/flight-computer` relative to `flight2/`).

## CLI flags

The binary also accepts global flags in addition to the runtime commands above:

- `--disable-file-logging` turns off on-disk vehicle-state logging.
- `--log-dirs <PATH>` writes logs to a custom directory instead of `$HOME/flight_logs`.
  - This flag is stackable. Repeat it to mirror logs to multiple directories.
- `--log-buffer-size <N>` changes the in-memory logging channel size. Default: `100`.
- `--log-rotation-mb <N>` rotates to a new log file once the current file reaches `N` MB. Default: `100`.
- `--fsync-rate <N>` forces buffered log data to sync to disk every `N` internal flushes. Default: `0` (disabled).
- `--print-gps` prints GPS data to the terminal at about 1 Hz.

Flags can be used on their own or combined with runtime commands such as `desktop`
or `disable-imu`.

### File logging examples

By default, file logging is enabled and writes to `$HOME/flight_logs`.

```bash
cargo run -p flight-computer --release
./target/release/flight-computer
```

Write logs to a specific directory:

```bash
cargo run -p flight-computer --release -- --log-dirs /home/ubuntu/flight_logs
./target/release/flight-computer --log-dirs /home/ubuntu/flight_logs
```

Write the same logs to multiple directories, such as both blackbox SD card
mounts on the Raspberry Pi:

```bash
cargo run -p flight-computer --release -- --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
./target/release/flight-computer --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
```

Rotate files at 25 MB instead of the default 100 MB:

```bash
cargo run -p flight-computer --release -- --log-rotation-mb 25
./target/release/flight-computer --log-rotation-mb 25
```

Increase the log buffer for bursty disk I/O:

```bash
cargo run -p flight-computer --release -- --log-buffer-size 500
./target/release/flight-computer --log-buffer-size 500
```

Disable file logging entirely:

```bash
cargo run -p flight-computer --release -- --disable-file-logging
./target/release/flight-computer --disable-file-logging
```

Combine logging flags with runtime commands:

```bash
cargo run -p flight-computer --release -- --log-dirs /home/ubuntu/flight_logs disable-imu disable-magnetometer
./target/release/flight-computer --log-dirs /home/ubuntu/flight_logs disable-imu disable-magnetometer
```

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
./target/release/flight-computer --log-dirs /mnt/blackbox_a --log-dirs /mnt/blackbox_b
```
