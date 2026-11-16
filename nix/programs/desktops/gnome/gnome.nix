{
  flake.modules.nixos.programs-desktops-gnome = {
    services.desktopManager.gnome.enable = true;
  };
}
