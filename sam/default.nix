{ crane, nixpkgs, self, ... }:
let
  inherit (nixpkgs) lib;

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

  # crossPkgs = pkgs.pkgsCross.armv7l-hf-multiplatform;
  #
  # flasher =
  #   let
  #     bootloader = self.packages.${system}.am335x-usb-bootloader;
  #   in
  #   pkgs.writeShellScriptBin "sam-flasher" ''
  #     exec ${self.packages.${system}.flash}/bin/flash bbone \
  #       --spl ${bootloader}/u-boot-spl.bin \
  #       --uboot ${bootloader}/u-boot.img \
  #       --image ${self.packages.${system}.sam.image}/sd-image/nixos-*.img \
  #       "$@"
  #   '';
in
{
  apps = pkgs:
    let
      crossPkgs = pkgs.pkgsCross.armv7l-hf-multiplatform;
      hostSystem = crossPkgs.stdenv.hostPlatform;
      buildSystem = crossPkgs.stdenv.buildPlatform;

      bootloader = self.packages.${hostSystem}.am335x-usb-bootloader;

      flasher = pkgs.writeShellScriptBin "sam-flasher" ''
        exec ${self.packages.${buildSystem}.flash}/bin/flash bbone \
          --spl ${bootloader}/u-boot-spl.bin \
          --uboot ${bootloader}/u-boot.img \
          --image ${self.packages.${hostSystem}.sam.image}/sd-image/nixos-*.img \
          "$@"
      '';
    in
    {
      sam.flash = {
        type = "app";
        program = "${flasher}/bin/sam-flasher";
      };
    };

  devShells.default = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };

  nixosConfigurations.sam =
    let
      overlay = (final: prev: {
        sam-runtime = self.packages.${prev.stdenv.hostPlatform}.sam.binary;
      });
    in
    lib.nixosSystem {
      modules = [
        ./build/release.nix
        { nixpkgs.overlays = [ overlay ]; }
      ];
    };

  packages = pkgs:
  let
    craneLib = (crane.mkLib pkgs).overrideToolchain pkgs.rustToolchain;
  in
  {
    sam = {
      binary = craneLib.buildPackage {
        pname = "sam";
        version = "1.0.0";
        src = craneLib.cleanCargoSource ../.;

        "${linkerEnvVar}" = "rust-lld";
        cargoExtraArgs = "-p sam --target ${targetTriple}";
        depsBuildBuild = [ pkgs.crossPkgs.armv7l-hf-multiplatform.stdenv.cc ];
      };

      image = self.nixosConfigurations.${pkgs.stdenv.buildSystem}.sam.config.system.build.sdImage;
    };
  };

  # packages.sam = {
  #   binary = craneLib.buildPackage {
  #     pname = "sam";
  #     version = "1.0.0";
  #     src = craneLib.cleanCargoSource ../.;
  #
  #     "${linkerEnvVar}" = "rust-lld";
  #     cargoExtraArgs = "-p sam --target ${targetTriple}";
  #     depsBuildBuild = [ crossPkgs.stdenv.cc ];
  #     doCheck = false;
  #   };
  #
  #   image = self.nixosConfigurations.${system}.sam.config.system.build.sdImage;
  # };
}
