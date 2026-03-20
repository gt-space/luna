{ ... }:
{
  imports = [
    ./rpi4.nix
  ];

  hardware.deviceTree = {
    filter = "bcm2711-rpi-cm4.dtb";
    name = "broadcom/bcm2711-rpi-cm4.dtb";
  };
}
