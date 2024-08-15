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

## Getting started for developement - macOS

### Install CLang and macOS Development Dependencies
Run the following command in your terminal:
```shell
xcode-select --install
```

### Install Node.js
Install from [here](https://nodejs.org/en/download/). Choose the macOS .pkg installer for 64-bit/ARM64.

### Install Rust
Run the following command in your terminal:
```shell
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
```
You might be prompted for your password during the Rust installation. If the installation was successful, the following line will appear:
```
Rust is installed now. Great!
```
Remember to restart your terminal after this for changes to take place.

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
