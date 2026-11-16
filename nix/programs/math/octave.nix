{
  flake.modules.homeManager.programs-math-octave = {pkgs, ...}: {
    home.packages = with pkgs; [octaveFull];
  };
}
