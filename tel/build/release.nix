{ lib, modulesPath, sx1280, ... }:
{
  imports = [
    "${modulesPath}/profiles/minimal.nix"
    ../../os/profiles/embedded.nix
    sx1280.nixosModules.default
  ];

  boot = {
    growPartition = true;

    kernel.sysctl."net.ipv4.ip_forward" = 1;
    kernelModules = [
      "gpio-dev"
      "i2c-dev"
      "spi_bcm2835"
      "spidev"
    ];

    supportedFilesystems = lib.mkForce [ "ext4" "vfat" ];
  };

  fileSystems = {
    "/" = {
      device = "/dev/disk/by-label/NIXOS_SD";
      fsType = "ext4";
      autoResize = true;
    };

    "/boot" = {
      device = "/dev/disk/by-label/FIRMWARE";
      fsType = "vfat";
      options = [ "fmask=0022" "dmask=0022" ];
    };
  };

  hardware = {
    bluetooth = {
      enable = false;
      powerOnBoot = false;
    };

    sx1280.enable = true;
  };

  networking = {
    dhcpcd.enable = false;
    firewall.enable = false;
    nftables.enable = true;
    useDHCP = false;
    useNetworkd = true;
    wireless.enable = false;
  };

  # Disable password requirements.
  security = {
    pam.services.sshd.allowNullPassword = true;
    sudo.wheelNeedsPassword = false;
  };

  # Permissive OpenSSH configuration for easy debugging.
  services.openssh = {
    enable = true;
    settings = {
      PermitEmptyPasswords = "yes";
      PermitRootLogin = "yes";
    };
  };

  # Original state version for TelemetryOS. Do not change.
  system.stateVersion = "25.11";

  systemd.network = {
    enable = true;
    networks = {
      "10-ethernet" = {
        matchConfig.Driver = "bcmgenet";
        networkConfig.Gateway = "192.168.1.1";
      };

      "20-radio0".matchConfig.Name = "radio0";
    };
  };

  # TODO: Consider switching the YJSP user to be purely in debug mode.
  # Technically, it should not be necessary in release mode, but may be
  # desirable for on-site debugging and testing.
  users = {
    groups.spi = {};

    motd = ''
      YJSP TelemetryOS v1.0.0
      Unauthorized access to this system is punishable by death.
    '';

    users.yjsp = {
      isNormalUser = true;
      password = "";
      extraGroups = [
        "dialout"
        "gpio"
        "i2c"
        "spi"
        "wheel"
      ];
    };
  };
}
