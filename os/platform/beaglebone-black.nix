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

  sdImage = {
    compressImage = false;

    populateFirmwareCommands =
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
    };
  };
}
