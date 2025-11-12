{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { flake-utils, nixpkgs, rust-overlay, ... } @ inputs:
  let
    overlays = [ (import rust-overlay) ];
  in
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs { inherit overlays system; };
      lib = pkgs.lib;

      rust = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
      crane = (inputs.crane.mkLib pkgs).overrideToolchain (_: rust);

      commonLibs = with pkgs; [
        dbus
        glib
        librsvg
        libsoup_2_4
        openssl
      ];

      linuxLibs = with pkgs; [
        at-spi2-atk
        atkmm
        cairo
        gdk-pixbuf
        gtk3
        harfbuzz
        libayatana-indicator
        libcanberra-gtk3
        pango
        webkitgtk_4_0
      ];

      libs = commonLibs
        ++ (lib.optionals pkgs.stdenv.isLinux linuxLibs);

      buildTools = with pkgs; [
        cargo-tauri
        gobject-introspection
        nodejs_24
        pkg-config
        rust
      ];
    in
    {
      devShells.default = pkgs.mkShell {
        buildInputs = libs;
        nativeBuildInputs = buildTools;
      };
    }
  );
}
