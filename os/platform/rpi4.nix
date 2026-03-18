# Configuration options applicable to all Raspberry Pi 4 platforms, namely the 4B and CM4.

{ modulesPath, nixos-hardware, ... }:
{
  imports = [
    "${modulesPath}/installer/sd-card/sd-image-aarch64.nix"
    ./rpi.nix
    nixos-hardware.nixosModules.raspberry-pi-4
  ];

  hardware.raspberry-pi."4".apply-overlays-dtmerge.enable = true;

  nixpkgs.hostPlatform = "aarch64-linux";

  sdImage.compressImage = false;
}
