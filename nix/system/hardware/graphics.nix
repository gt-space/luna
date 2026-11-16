{
  flake.modules.nixos.system-hardware-graphics = {
    hardware.graphics = {
      enable = true;
      enable32Bit = true;
    };
  };
}
