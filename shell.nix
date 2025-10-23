{ pkgs ? import <nixpkgs> {} }:
let
  overrides = (builtins.fromTOML (builtins.readFile ./rust-toolchain.toml));
in
  pkgs.mkShell rec {
    buildInputs = with pkgs; [
      cargo-tauri
      clang
      cmake
      dbus
      hdf5
      llvmPackages.bintools
      nodejs_23
      openssl
      pkg-config
      rustup
      webkitgtk_4_0
    ];

    RUSTC_VERSION = overrides.toolchain.channel;
    LIBCLANG_PATH = pkgs.lib.makeLibraryPath [ pkgs.llvmPackages.libclang.lib ];
    TMPDIR="/tmp";
  }
