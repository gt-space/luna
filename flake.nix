{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { flake-utils, nixpkgs, rust-overlay, ... }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ (import rust-overlay) ];
      };

      rustToolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = [ "rust-analyzer" ];
        targets = [ "armv7-unknown-linux-musleabihf" ];
      };
    in
    {
      devShells.default = pkgs.mkShell {
        nativeBuildInputs = with pkgs; [
          clang
          cmake
          dbus
          hdf5
          llvmPackages.bintools
          nodejs_24
          openssl
          pkg-config
          rustToolchain
          libsoup_3
          webkitgtk_4_1
        ];
      };
    }
  );
}
