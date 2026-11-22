{
  flake.modules.homeManager.programs-networking-lsusb = {pkgs, ...}: {
    home.packages = with pkgs; [usbutils];
  };
}
