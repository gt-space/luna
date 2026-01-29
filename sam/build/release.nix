{ lib, modulesPath, pkgs, ... }:
{
  imports = [
    ../../os/platform/beaglebone-black.nix
    "${modulesPath}/profiles/minimal.nix"
  ];

  boot = {
    enableContainers = false;
    growPartition = true;

    initrd = {
      includeDefaultModules = false;
      availableKernelModules = [ "mmc_block" "ext4" ];
    };

    tmp.cleanOnBoot = true;
    loader.timeout = 0;
  };

  documentation = {
    enable = false;
    doc.enable = false;
    info.enable = false;
    man.enable = false;
    nixos.enable = false;
  };

  environment.defaultPackages = lib.mkForce [ ];

  networking = {
    defaultGateway = "192.168.1.1";
    firewall.enable = false;
    interfaces.eth0.useDHCP = false;
    wireless.enable = false;
  };

  programs.command-not-found.enable = false;

  security = {
    pam.services.sshd.allowNullPassword = true;
    sudo.wheelNeedsPassword = false;
  };

  services.openssh = {
    enable = true;
    settings = {
      PermitEmptyPasswords = "yes";
      PermitRootLogin = "yes";
    };
  };

  system = {
    disableInstallerTools = true;
    extraDependencies = lib.mkForce [ ];
    stateVersion = "25.11";
  };
}
