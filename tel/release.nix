{ lib, modulesPath, nixos-hardware, pkgs, sx1280, version, ... }:
{
  imports = [
    "${modulesPath}/profiles/minimal.nix"
    nixos-hardware.nixosModules.raspberry-pi-4
    sx1280.nixosModules.default
  ];

  boot = {
    enableContainers = false;
    growPartition = true;
    kernelPackages = pkgs.linuxPackages_rpi4;
    tmp.cleanOnBoot = true;
    initrd.allowMissingModules = true;
    loader.timeout = 0;

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

  environment = {
    defaultPackages = lib.mkForce [ ];
    etc = {
      "ssh/ssh_host_ed25519_key" = {
        mode = "0600";
        source = ./keys/ed25519.pem;
      };

      "ssh/ssh_host_ed25519_key.pub" = {
        mode = "0600";
        source = ./keys/ed25519.pub;
      };

      "ssh/ssh_host_rsa_key" = {
        mode = "0600";
        source = ./keys/rsa.pem;
      };

      "ssh/ssh_host_rsa_key.pub" = {
        mode = "0600";
        source = ./keys/rsa.pub;
      };
    };
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

  hardware.deviceTree = {
    dtbSource = pkgs.device-tree_rpi;
    enable = true;
    filter = "bcm2711-rpi-4-b.dtb";
    name = "broadcom/bcm2711-rpi-4-b.dtb";
    overlays = [
      {
        dtsFile = ./overlays/sx1280.dtso;
        name = "sx1280.dtbo";
      }
    ];
  };

  hardware = {
    bluetooth = {
      enable = false;
      powerOnBoot = false;
    };

    enableRedistributableFirmware = lib.mkForce false;
    raspberry-pi."4".apply-overlays-dtmerge.enable = true;

    sx1280 = {
      enable = true;
      dtso = ./overlays/sx1280.dtso;
    };
  };

  networking = {
    defaultGateway = "192.168.1.1";
    firewall.enable = false;
    hostName = "tel";
    nameservers = [ "1.1.1.1" "8.8.8.8" ];

    interfaces.eth0 = {
      useDHCP = false;
      ipv4.addresses = [
        {
          address = "192.168.2.132";
          prefixLength = 24;
        }
      ];
    };

    wireless.enable = false;
  };

  # Disable Nix, as it's not necessary in a deployed state and takes up
  # resources. This option is re-enabled in debug mode.
  nix.enable = false;

  programs = {
    bash.promptInit = ''
      export PS1="\u@\h (version ${version}) $ "
    '';

    command-not-found.enable = false;
  };

  # Disable password requirements.
  security = {
    pam.services.sshd.allowNullPassword = true;
    sudo.wheelNeedsPassword = false;
  };

  # Permissive OpenSSH configuration for easy debugging.
  services = {
    openssh = {
      enable = true;
      settings = {
        PermitEmptyPasswords = "yes";
        PermitRootLogin = "yes";
      };
    };

    udev.extraRules = ''
      SUBSYSTEM=="spidev", KERNEL=="spidev0.0", GROUP="spi", MODE="0660"
    '';
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
