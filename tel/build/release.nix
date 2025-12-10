{ lib, modulesPath, sx1280, ... }:
{
  imports = [
    "${modulesPath}/profiles/minimal.nix"
    sx1280.nixosModules.default
  ];

  boot = {
    enableContainers = false;
    growPartition = true;
    tmp.cleanOnBoot = true;
    loader.timeout = 0;

    kernel.sysctl."net.ipv4.ip_forward" = 1;
    kernelModules = [
      "gpio-dev"
      "i2c-dev"
      "spi_bcm2835"
      "spidev"
    ];

    supportedFilesystems = lib.mkForce [ "ext4" "vfat" ];
  };

  console.enable = false;

  # Disable all documentation.
  documentation = {
    enable = false;
    doc.enable = false;
    info.enable = false;
    man.enable = false;
    nixos.enable = false;
  };

  environment.defaultPackages = lib.mkForce [ ];

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

    enableRedistributableFirmware = lib.mkForce false;
    sx1280.enable = true;
  };

  networking = {
    defaultGateway = "192.168.1.1";
    firewall.enable = false;
    interfaces = {
      eth0.useDHCP = false;
      radio0.useDHCP = false;
    };
    wireless.enable = false;
  };

  # Disable Nix, as it's not necessary in a deployed state and takes up
  # resources. This option is re-enabled in debug mode.
  nix.enable = false;

  programs.command-not-found.enable = false;

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

  system = {
    # Disable installer tools, since this is not an installer image.
    disableInstallerTools = true;

    # Disable extra dependencies (that won't be used anyway).
    extraDependencies = lib.mkForce [ ];

    # Original state version for TelemetryOS. Do not change.
    stateVersion = "25.11";

    # Disable switching configurations.
    switch.enable = false;
  };

  # Enable compressed RAM as an alternative to swapping to disk (fast).
  zramSwap = {
    enable = true;
    memoryPercent = 20;
  };

  # TODO: Consider switching the YJSP user to be purely in debug mode.
  # Technically, it should not be necessary in release mode, but may be
  # desirable for on-site debugging and testing.
  users = {
    groups.spi = {};

    motd = ''
      YJSP TelemetryOS v[TODO]
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

  xdg = {
    autostart.enable = false;
    icons.enable = false;
    mime.enable = false;
    sounds.enable = false;
  };
}
