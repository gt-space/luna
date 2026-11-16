{
  flake.modules.nixos.programs-networking-openssh = {
    services.openssh = {
      enable = true;

      settings.PasswordAuthentication = false;
    };
  };
}
