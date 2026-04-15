{ lib, modulesPath, pkgs, ... }:
{
  imports = [
    "${modulesPath}/profiles/minimal.nix"
  ];

  boot = {
    enableContainers = false;
    initrd.includeDefaultModules = false;
    loader.timeout = 0;
    tmp.cleanOnBoot = true;
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
    defaultPackages = lib.mkForce (with pkgs; [
      bash
      coreutils
    ]);

    # Ensure no packages leak into system-path.
    systemPackages = lib.mkForce [ ];
  };

  fonts.fontconfig.enable = false;
  hardware.enableRedistributableFirmware = lib.mkForce false;
  i18n.supportedLocales = [ "en_US.UTF-8/UTF-8" ];

  nix = {
    # Prevent the nixpkgs source tree from being placed in the store.
    nixPath = lib.mkForce [ ];
    registry = lib.mkForce { };
    settings.auto-optimise-store = true;
  };

  programs.command-not-found.enable = false;

  security = {
    audit.enable = false;
    apparmor.enable = false;
    polkit.enable = false;
  };

  system = {
    disableInstallerTools = true;
    extraDependencies = lib.mkForce [ ];
  };

  xdg = {
    autostart.enable = false;
    icons.enable = false;
    mime.enable = false;
    sounds.enable = false;
  };

  # Enable compressed RAM as an alternative to swapping to disk (fast).
  zramSwap = {
    enable = true;
    memoryPercent = 20;
  };
}
