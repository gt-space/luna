{
  flake.modules.homeManager.programs-camera-shotwell = {pkgs, ...}: {
    home.packages = with pkgs; [shotwell];
  };
}
