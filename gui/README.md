# fs-gui

## Getting started for developement - Ubuntu

### Install necessary packages
```bash
sudo apt install libwebkit2gtk-4.0-dev \
    build-essential \
    curl \
    wget \
    libssl-dev \
    libgtk-3-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev
```

### Install Rust
```bash
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
```

### Install Node.js
```bash
curl -fsSL https://deb.nodesource.com/setup_19.x | sudo -E bash - &&\
sudo apt-get install -y nodejs
```

### Install Yarn
```bash
sudo npm install -g yarn
```

### Run developement server
```bash
cd fs-gui
yarn tauri dev
```

## Getting started for developement - Windows

### Install Microsoft Visual Studio C++ Build Tools
Install from [here](https://visualstudio.microsoft.com/visual-cpp-build-tools/). When asked which workloads to install, **ensure "C++ build tools" and the Windows 10 SDK are selected**.

### Install Node.js
Install from [here](https://nodejs.org/en/download/). Choose the Windows .msi installer for your system's architecture.

### Install Rust
Install from [here](https://www.rust-lang.org/tools/install). Choose the rustup option and select the installer for your system's architecture. This will not work if you don't have C++ Build Tools installed. You will have to restart your terminal after this for changes to show up.

### Install Yarn
In your terminal, run:
```shell
npm install --global yarn
```

### Run developement server
After cloning this repository, install dependencies using:
```shell
cd fs-gui
npm install -g package.json
```
To run the development server:
```shell
npm run tauri dev
```

## Recommended IDE Setup

- [VS Code](https://code.visualstudio.com/) + [Tauri](https://marketplace.visualstudio.com/items?itemName=tauri-apps.tauri-vscode) + [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer)

## Debugging in VS Code
A good guide on how to debug Tauri apps in VS Code can be found [here](https://tauri.app/v1/guides/debugging/vs-code).

## CTV operations page
Open it from the taskbar: **Views > CTV**. It replaces writing Python sequences to arm the engine controller (EC) and start a hopper flight. The flight computer must be running the CTV flight logic (`--gnc-gains`), the GUI must be connected to Servo, and the telemetry source must be **Umbilical** (the radio link does not carry CTV status; the page warns and offers a switch).

Intended operator flow, one primary action at a time:
1. Check the **Pre-flight checks** list. Every check must be green for the **READY** indicator.
2. **Arm EC** gives the engine controller output authority.
3. **START FLIGHT** (enabled once the EC is armed and READY). It asks for a second click to confirm.
4. While flying the primary button becomes **Abort / Land**: the vehicle flies down from its current altitude. No confirmation, it is the time-critical action.
5. After landing, **Disarm EC**. If the EC was safed, the primary action is **Clear safing**.

**TERMINATE** is always visible (red, confirms). It safes the EC immediately and latches it; it is not the same as Abort / Land, which keeps demanding thrust. **Disarm EC** is not offered in flight.

**Re-initialize navigation** (a secondary button on the pad, after a landing and when safed) throws the navigation filter away and levels it again, as at start-up, with the vehicle held still: the estimate is unavailable for a few seconds (the Navigation estimate check shows *Leveling*) and then healthy. It is the way back from an unhealthy estimate such as *GPS rejected repeatedly*, which otherwise stays unhealthy until the flight computer is restarted. It is refused in flight, where the controller is flying on the estimate.

Disabled buttons say why underneath (and in the tooltip). Every command shows *pending* (sending to Servo), *sent* (Servo forwarded it), *acknowledged* (the status shows the effect) or *failed* (Servo refused, or no effect within 5 s).

The status panel shows the mode, mission time against the end of the gain schedule (a marker shows the schedule position when an abort to land has moved it), altitude, tilt, speed, EC state and safing reason, gains, and missed control periods (highlighted for 15 s after an increase). A latched fault is a red banner. Telemetry is flagged stale from Servo's transport stats (older than 1.5 s), because Servo re-sends the last frame at 10 Hz even when the flight computer is silent; values are dimmed then, and Start/Arm are disabled, but Abort and Terminate stay available.

The event log records every change of the flight logic's last event, mode, fault, EC state and missed periods, the operator's own commands and their outcome, and telemetry loss. It lives in the window only: it is gone when the window is closed, so keep it open during a flight.

### Plots

Below the status there are two tabs of plots, drawn with Chart.js from the control periods the flight computer sends with its status (a sliding 200 ms window, because servo forwards only the newest frame ten times a second; the page drops what it has already seen). They show the latest flight from its start (x is time since the mission start, with a little of the pad before it), or the last minute when there is no flight in the window. The history lives in the window only, like the event log: **Clear** empties it, and a restart of the mission clock starts a new flight.

*Navigation filter*
- The **filter health** and, per sensor, whether its latest reading was taken or rejected (barometer, GPS, magnetometer, four LiDARs).
- The **estimated position and velocity** (north, east, up, in the filter's frame from the pad), the **tilt** and the **body rates**.
- The **uncertainty** (1 sigma) of position and velocity, and of tilt and heading: it grows without readings and drops when they are taken.
- The **GPS innovation** (normalized, against the gate of 16.8): readings above the line are rejected, and many in a row make the filter unhealthy.

*Controller* (while flying)
- **Altitude** as the controller was given it against the schedule it tracks, and the **position and velocity errors**.
- The **commanded thrust**, the **commanded TVC angle** with the gimbal limit drawn (a command that sits on the line has run out of authority), and the **roll torque** it asks for.

Logic tests (no browser or Tauri needed): `npx playwright test tests/ctv-logic.spec.ts tests/ctv-trace.spec.ts`.
