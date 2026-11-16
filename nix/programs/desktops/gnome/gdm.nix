{
  flake.modules.nixos.programs-desktops-gnome = {
    services.displayManager.gdm.enable = true;
  };
}
