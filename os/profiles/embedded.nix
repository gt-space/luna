{ lib, modulesPath, ... }:
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

  environment.defaultPackages = lib.mkForce [ ];

  fonts.fontconfig.enable = false;

  i18n.supportedLocales = [ "en_US.UTF-8/UTF-8" ];

  nix = {
    # Prevent the nixpkgs source tree from being placed in the store.
    nixPath = lib.mkForce [ ];
    registry = lib.mkForce { };
    settings.auto-optimise-store = true;
  };

  programs.command-not-found.enable = false;

  # Ensure no packages leak into system-path
  environment.systemPackages = lib.mkForce [ ];

  security = {
    audit.enable = false;
    apparmor.enable = false;
    polkit.enable = false;
  };

  system = {
    disableInstallerTools = true;
    extraDependencies = lib.mkForce [ ];
  };
}
