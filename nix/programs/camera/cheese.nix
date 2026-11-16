{
  flake.modules.homeManager.programs-camera-cheese = {pkgs, ...}: {
    home.packages = with pkgs; [cheese];
  };
}
