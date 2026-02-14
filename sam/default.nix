{ craneLib, lib, pkgs, self, system, ... }:
let
  # The universal target triple for all SAM deployments (Beaglebone Black).
  targetTriple = "armv7-unknown-linux-musleabihf";
  linkerEnvVar = "CARGO_TARGET_${
    lib.toUpper (
      builtins.replaceStrings
        [ "-" ]
        [ "_" ]
        targetTriple
    )
  }_LINKER";

  crossPkgs = pkgs.pkgsCross.armv7l-hf-multiplatform;
in
{
  devShells.default = pkgs.mkShell {
    packages = with pkgs; [ rustToolchain ];
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
