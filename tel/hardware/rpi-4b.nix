{ nixos-hardware, pkgs, ... }:
{
  imports = [
    nixos-hardware.nixosModules.raspberry-pi-4
  ];

  boot = {
    kernelPackages = pkgs.linuxPackages_rpi4;
    initrd.allowMissingModules = true;
  };

  hardware.deviceTree = {
    dtbSource = pkgs.device-tree_rpi;
    enable = true;
    filter = "bcm2711-rpi-4-b.dtb";
    name = "broadcom/bcm2711-rpi-4-b.dtb";
  };

  hardware.raspberry-pi."4".apply-overlays-dtmerge.enable = true;
}
