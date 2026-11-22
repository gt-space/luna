{
  flake.modules.nixos.programs-networking-avahi = {
    services.avahi = {
      enable = true;
      nssmdns4 = true;
    };
  };
}
