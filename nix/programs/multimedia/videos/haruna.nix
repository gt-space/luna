{
  flake.modules.homeManager.programs-multimedia-videos-haruna = {pkgs, ...}: {
    home.packages = with pkgs; [haruna];
  };
}
