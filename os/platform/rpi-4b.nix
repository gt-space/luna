{ ... }:
{
  imports = [
    ./rpi4.nix
  ];

  hardware.deviceTree = {
    filter = "bcm2711-rpi-4-b.dtb";
    name = "broadcom/bcm2711-rpi-4-b.dtb";
  };
}
