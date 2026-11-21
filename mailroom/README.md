# Mailroom

Mailroom opens flight-computer `.postcard` logs on an operator laptop. Use it to review samples, combine chunks from a run, and save the combined timeline as CSV.

## Run on a laptop

Use Node.js 22 or 24 LTS for a new setup. Mailroom also works with Node.js 20 in existing setups. Install Rust through `rustup` so it picks up the repository toolchain. Follow the [Tauri desktop prerequisites](https://v2.tauri.app/start/prerequisites/) for your OS. Windows needs the MSVC C++ build tools and WebView2. macOS needs the Xcode command-line tools and a current system WebView. Ubuntu 22.04 or newer needs WebKitGTK 4.1 and the other Debian-based Linux prerequisites.

From the repository root, run these commands. `npm ci` is needed after a fresh clone or a dependency update.

```sh
cd mailroom
npm ci
npm run tauri dev
```

Mailroom uses local port 1421. For optional internal bundles, use `npm run tauri build -- --bundles nsis` on Windows, `npm run tauri build -- --bundles app` on macOS, or `npm run tauri build -- --bundles deb` on Ubuntu. These commands use native platform build tools and default signing settings.

## Open or download a run

- Drop one or more `.postcard` files into the window, or use **Open files**. Use **Add chunks** to extend the timeline or **Open new run** to replace it. The **Source files** list shows rotation gaps, overlapping time ranges, and incomplete records.
- Search **Columns** for fields such as `gps`. Use **Clear selection** and then check the fields you want to see. Turn pages with **Previous** and **Next**, or type a page number and press Enter.
- Choose **Export all columns** or **Export visible columns**. Both export every row in the current timeline. The `timestamp` column uses Unix seconds, and missing values appear as empty cells.
- Choose **Connect to FC** to download logs. The defaults are `ubuntu@flight.local` and `/home/ubuntu/flight_logs`. Sign in with the FC password, select the chunks, then choose **Download & open** and a laptop folder. If the `.local` hostname fails to resolve on your laptop, enter the Pi's IP address.

On first connection, compare the SHA-256 fingerprint with a trusted copy from the FC operator. Mailroom remembers the approved fingerprint for that host and port. If the Pi was reimaged or its SSH keys rotated, verify the new fingerprint with the FC operator, then close Mailroom and remove the old host entry in `flight-computer.json` under the app config directory for `org.yjsp.mailroom`. The password is entered for each connection.
