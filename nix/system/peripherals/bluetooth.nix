{
  flake.modules.nixos.system-peripherals-bluetooth = {
    hardware.bluetooth = {
      enable = true;
      powerOnBoot = true;
    };
  };
}
