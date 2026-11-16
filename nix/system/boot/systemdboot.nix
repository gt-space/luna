{
  flake.modules.nixos.system-boot-systemdboot = {
    boot.loader = {
      systemd-boot.enable = true;
      efi.canTouchEfiVariables = true;
    };
  };
}
