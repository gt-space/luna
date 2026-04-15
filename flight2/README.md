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

### Desktop mode (no FC-local SPI sensor workers)

Use the `desktop` subcommand to skip starting the MAG+BAR and IMU+ADC workers (useful on a dev machine without that hardware):

```bash
cargo run -p flight-computer --release -- desktop
```

Global options still work after the subcommand, for example:

```bash
cargo run -p flight-computer --release -- desktop --print-gps
```

### Build only, then run

`cargo build` does not accept program arguments like `desktop`; it only compiles.

```bash
cargo build -p flight-computer --release
```

Then run the release binary from the workspace root (artifact path is under `target/release/`):

```bash
./target/release/flight-computer
./target/release/flight-computer desktop
```

If you built from inside `flight2/`, the binary is still emitted at the workspace `target/` when this crate is part of the workspace (default layout: `../target/release/flight-computer` relative to `flight2/`).