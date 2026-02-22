{ config, lib, modulesPath, pkgs, ... }:
{
  imports = [
    "${modulesPath}/installer/sd-card/sd-image.nix"
  ];

  boot = {
    initrd.availableKernelModules = [ "mmc_block" "ext4" ];
    kernelParams = [ "console=ttyO0,115200n8" ];
    kernelPackages = pkgs.linuxPackages_6_18;
    kernelPatches = [{
      name = "bbb-minimize";
      patch = null;
      structuredExtraConfig = lib.mapAttrs (_: lib.mkForce) (with lib.kernel; {
        WLAN = no;
        WIRELESS = no;
        CFG80211 = no;
        BLUETOOTH = no;
        SOUND = no;
        SND = no;
        DRM = no;
        FB = no;
        VT = no;
        USB_STORAGE = no;
        INPUT_JOYSTICK = no;
        INPUT_TABLET = no;
        INPUT_TOUCHSCREEN = no;
        NFS_FS = no;
        NFSD = no;
        CIFS = no;
        REISERFS_FS = no;
        JFS_FS = no;
        XFS_FS = no;
        BTRFS_FS = no;
        NTFS_FS = no;
        CAN = no;
        MEDIA_SUPPORT = no;
        RC_CORE = no;
        INFINIBAND = no;
        ISDN = no;
        ATM = no;
        PCMCIA = no;
        ATA = no;
        SCSI = no;
        FIREWIRE = no;
        SPI = yes;
        SPI_OMAP2_MCSPI = yes;
      });
    }];

    supportedFilesystems = lib.mkForce [ "ext4" ];

    loader = {
      generic-extlinux-compatible.enable = true;
      grub.enable = false;
    };
  };

  hardware = {
    deviceTree = {
      enable = true;
      name = "am335x-boneblack.dtb";
    };

    enableAllHardware = lib.mkForce false;
  };

  networking.wireless.enable = false;

  nixpkgs.crossSystem.config = "armv7l-unknown-linux-gnueabihf";

  sdImage =
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

      dd = "${pkgs.buildPackages.coreutils}/bin/dd";
    in
    {
      compressImage = false;

      # The bootloader is placed into raw offsets, so no firmware partition is
      # necessary.
      firmwareSize = 1;
      populateFirmwareCommands = lib.mkForce "";

      # Write the bootloader at raw offsets in the gap before the firmware
      # partition.  The AM335x ROM loads SPL (MLO) from 0x20000 (128 KiB)
      # in raw mode.  The SPL then loads u-boot.img from 0x60000 (384 KiB).
      postBuildCommands = ''
        ${dd} if=${uboot}/MLO of=$img bs=128k seek=1 conv=notrunc
        ${dd} if=${uboot}/u-boot.img of=$img bs=128k seek=3 conv=notrunc
      '';

      populateRootCommands = ''
        mkdir -p ./files/boot
        ${config.boot.loader.generic-extlinux-compatible.populateCmd} \
          -c ${config.system.build.toplevel} \
          -d ./files/boot
      '';
    };

  systemd = {
    coredump.enable = false;

    package = pkgs.systemd.override {
      # Core fixes
      withBootloader = false;
      withEfi = false;

      withCryptsetup = false;
      withGcrypt = false;

      withDocumentation = false;
      withFido2 = false;
      withTpm2Tss = false;
      withHomed = false;
      withImportd = false;
      withMachined = false;
      withPortabled = false;
      withRemote = false;
      withRepart = false;
      withShellCompletions = false;
      withAnalyze = false;
      withLocaled = false;
      withTimedated = false;
      withHostnamed = false;
      withUserDb = false;
      withCoredump = false;
      withSysupdate = false;
    };
  };
}
