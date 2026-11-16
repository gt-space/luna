{
  flake.modules.homeManager.programs-networking-wget = {pkgs, ...}: {
    home.packages = with pkgs; [wget];
  };
}
