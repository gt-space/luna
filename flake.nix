{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    crane.url = "github:ipetkov/crane";
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { flake-utils, nixpkgs, rust-overlay, ... } @ inputs:
  let
    inherit (nixpkgs) lib;

    projectPaths = [
      ./sam
    ];

    projectOutputs = map (path: import path inputs) projectPaths;
  in
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ (import rust-overlay) ];
      };

      rustToolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = [ "rust-src" "rust-analyzer" ];
        targets = [ "armv7-unknown-linux-musleabihf" ];
      };
    in
    {
      devShells.default = pkgs.mkShell {
        packages = with pkgs; [
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
  )
  // lib.foldr lib.recursiveUpdate { } projectOutputs;
}
