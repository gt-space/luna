{ self, crane, flake-utils, nixpkgs, rust-overlay, ... }:
let
  inherit (nixpkgs) lib;
in
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs {
      inherit system;
      overlays = [ (import rust-overlay) ];
    };

    targetTriple = "armv7-unknown-linux-musleabihf";
    linkerEnvVar = "CARGO_TARGET_${
      lib.toUpper
      (builtins.replaceStrings
        ["-"]
        ["_"]
        targetTriple
      )
    }_LINKER";

    rustToolchain = pkgs.rust-bin.stable."1.93.0".default.override {
      targets = [ targetTriple ];
    };

    craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;
    crossPkgs = pkgs.pkgsCross.armv7l-hf-multiplatform;
  in
  {
    devShells.default = pkgs.mkShell {
      nativeBuildInputs = [
        rustToolchain
        crossPkgs.stdenv.cc
      ];
    };

    nixosConfigurations.sam = lib.nixosSystem {
      inherit system;
      modules = [ ./build/release.nix ];
    };

    packages.sam = {
      binary = craneLib.buildPackage {
        pname = "sam";
        version = "1.0.0";
        src = craneLib.cleanCargoSource ../.;

        "${linkerEnvVar}" = "rust-lld";
        cargoExtraArgs = "-p sam --target ${targetTriple}";
        depsBuildBuild = [ crossPkgs.stdenv.cc ];
        doCheck = false;
      };

      image = self.nixosConfigurations.${system}.sam.config.system.build.sdImage;
    };
  }
)
