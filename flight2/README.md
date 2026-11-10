# Flight computer (`flight-computer`)

Rust binary for the flight computer control loop. Crate package name: `flight-computer`.

## Abort-stage persistence

Flight reserves a new revision epoch at each start in `LUNA_ABORT_EPOCH_DIR`
(default `/var/lib/luna/abort-revisions`). The same persistent directory holds
`required-sams`, a checksummed inventory of SAMs that must acknowledge every new
stage. Updates write and sync a temporary snapshot, atomically replace the
inventory, then sync its directory. A crash before replacement leaves the old
inventory intact; a damaged snapshot blocks confirmation.

For first-time provisioning, the previous two-file format is still accepted:
create an empty `required-sams` and `required-sams-state` containing
`0000000000000000cbf29ce484222325` followed by a newline. On the first stage
request, Flight migrates these files to the atomic snapshot in `required-sams`;
the old state file is ignored thereafter. Provision **only before the first use
of revisioned abort stages**, after verifying that no SAM has an existing
configuration to recover. Never replace or empty the inventory to retire a SAM.

The directory and inventory must survive process and system restarts and be
writable by the Flight service account. On the target, check them with:

```bash
namei -l /var/lib/luna/abort-revisions/required-sams
sudo -u <flight-user> test -r /var/lib/luna/abort-revisions/required-sams
sudo -u <flight-user> test -w /var/lib/luna/abort-revisions
```

Each check must succeed. Before migration, the service must also be able to
read `required-sams-state`. After a controlled Flight restart and system reboot,
confirm that `required-sams` contains all recorded IDs, a higher `epoch-*` file exists, and
Flight remains UNSYNCHRONIZED until every recorded SAM acknowledges a fresh
transition. Missing or damaged storage blocks stage confirmation; emergency
abort remains available. This repository packages Flight but does not define
its runtime service or provision its target filesystem.

The new millisecond-precision `PrepareAbortStageMillis` command must be supported
by the deployed SAM firmware. Older SAMs will not acknowledge it; update Flight
and SAM together and confirm the wire-format roundtrip before hardware use.

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
