{ lib, modulesPath, pkgs, ... }:
{
  imports = [
    "${modulesPath}/installer/sd-card/sd-image-armv7l-multiplatform.nix"
  ];

  boot = {
    kernelParams = [ "console=ttyS0,115200n8" ];
    loader.generic-extlinux-compatible.enable = true;
  };

  hardware.deviceTree = {
    enable = true;
    name = "am335x-boneblack.dtb";
  };

  nixpkgs = {
    config.allowBroken = true;
    crossSystem.config = "armv7l-unknown-linux-gnueabihf";
  };

  sdImage.populateFirmwareCommands =
    let
      # Build a version of U-Boot specifically compatible with the Beaglebone
      # Black (which uses the AM335x), with the assumption that it will be
      # flashed directly to the eMMC.
      uboot = pkgs.buildUBoot {
        defconfig = "am335x_evm_defconfig";
        extraMeta.platforms = [ "armv7l-linux" ];
        filesToInstall = [ "MLO" "u-boot.img" ];

        # Disable non-eMMC boot options to save space on the tiny boot sector.
        extraConfig = ''
          CONFIG_SPL_NAND_SUPPORT=n
          CONFIG_SPL_MTD_SUPPORT=n
          CONFIG_SPL_SPI_SUPPORT=n
          CONFIG_SPL_NET_SUPPORT=n
        '';
      };

      cp = "${pkgs.buildPackages.coreutils}/bin/cp";
    in
    lib.mkForce ''
      ${cp} ${uboot}/MLO firmware/MLO
      ${cp} ${uboot}/u-boot.img firmware/u-boot.img
    '';

  systemd.package = pkgs.systemdMinimal.override {
    withBootloader = false;
    withEfi = false;
  };
}
