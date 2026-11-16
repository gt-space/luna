{
  flake.modules.homeManager.programs-compression-p7zip = {pkgs, ...}: {
    home.packages = with pkgs; [p7zip];
  };
}
