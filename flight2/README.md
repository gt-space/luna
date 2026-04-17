# Flight computer (`flight-computer`)

Rust binary for the flight computer control loop. Crate package name: `flight-computer`.

## Running

From the **workspace root** (`luna/`):

```bash
cargo run -p flight-computer --release
```

From `**flight2/**`:

```bash
cargo run --release
```

### Disable FC-local sensors

The flight computer accepts stackable command words to disable individual
workers and FC-local sensors without touching their hardware interfaces:

```bash
cargo run -p flight-computer --release -- disable-gps
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

`cargo build` does not accept program arguments like `desktop`; it only compiles.

```bash
cargo build -p flight-computer --release
```

Then run the release binary from the workspace root (artifact path is under `target/release/`). A few examples of some subcommands that you can run the binary with are also listed:

```bash
./target/release/flight-computer
./target/release/flight-computer desktop
./target/release/flight-computer disable-gps disable-imu
./target/release/flight-computer disable-imu disable-magnetometer
```

If you built from inside `flight2/`, the binary is still emitted at the workspace `target/` when this crate is part of the workspace (default layout: `../target/release/flight-computer` relative to `flight2/`).