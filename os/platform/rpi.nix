# Configuration options universally applicable to all Raspberry Pi platforms.

{ lib, pkgs, ... }:
{
  boot = {
    initrd.allowMissingModules = true;
    supportedFilesystems = lib.mkForce [ "ext4" "vfat" ];
  };

  hardware.deviceTree = {
    dtbSource = pkgs.device-tree_rpi;
    enable = true;
  };
}
