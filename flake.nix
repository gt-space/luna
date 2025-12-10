{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-hardware.url = "github:NixOS/nixos-hardware";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    sx1280.url = "github:jeffcshelton/sx1280";
  };

  outputs = { nixpkgs, ... } @ inputs:
  let
    inherit (nixpkgs) lib;

    projectPaths = [
      ./tel
    ];

    projectOutputs = map (path: import path inputs) projectPaths;
  in
  lib.foldr lib.recursiveUpdate { } projectOutputs;
}
#   flake-utils.lib.eachDefaultSystem (system:
#     let
#       pkgs = import nixpkgs { inherit overlays system; };
#       lib = pkgs.lib;
#
#       rust = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
#       crane = (inputs.crane.mkLib pkgs).overrideToolchain (_: rust);
#
#       commonLibs = with pkgs; [
#         dbus
#         glib
#         librsvg
#         libsoup_2_4
#         openssl
#       ];
#
#       linuxLibs = with pkgs; [
#         at-spi2-atk
#         atkmm
#         cairo
#         gdk-pixbuf
#         gtk3
#         harfbuzz
#         libayatana-indicator
#         libcanberra-gtk3
#         pango
#         webkitgtk_4_0
#       ];
#
#       libs = commonLibs
#         ++ (lib.optionals pkgs.stdenv.isLinux linuxLibs);
#
#       buildTools = with pkgs; [
#         cargo-tauri
#         gobject-introspection
#         nodejs_24
#         pkg-config
#         rpiboot
#         rust
#       ];
#     in
#     {
#       apps.tel = tel.apps.${system};
#
#       devShells.default = pkgs.mkShell {
#         buildInputs = libs;
#         nativeBuildInputs = buildTools;
#       };
#
#       packages.tel = tel.packages.${system};
#     }
#   );
