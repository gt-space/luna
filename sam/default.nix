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

  flasher =
    let
      bootloader = self.packages.${system}.am335x-usb-bootloader;
    in
    pkgs.writeShellScriptBin "sam-flasher" ''
      exec ${self.packages.${system}.flash}/bin/flash bbone \
        --spl ${bootloader}/u-boot-spl.bin \
        --uboot ${bootloader}/u-boot.img \
        --image ${self.packages.${system}.sam.image}/sd-image/nixos-*.img \
        "$@"
    '';
in
{
  apps.sam.flash = {
    type = "app";
    program = "${flasher}/bin/sam-flasher";
  };

  devShells.default = pkgs.mkShell {
    packages = with pkgs; [ rustToolchain ];
  };

  nixosConfigurations.sam =
    let
      overlay = (final: prev: {
        sam-runtime = self.packages.${system}.sam.binary;
      });
    in
    lib.nixosSystem {
      inherit system;
      modules = [
        ./build/release.nix
        { nixpkgs.overlays = [ overlay ]; }
      ];
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
