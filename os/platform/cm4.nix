{ config, lib, modulesPath, nixos-hardware, pkgs, ... }:
{
  imports = [
    nixos-hardware.nixosModules.raspberry-pi-4
  ];

  nixpkgs.hostPlatform = "aarch64-linux";

  sdImage.compressImage = false;
}
