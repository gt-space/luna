{
  flake.modules.homeManager.programs-compression-xz = {pkgs, ...}: {
    home.packages = with pkgs; [xz];
  };
}
