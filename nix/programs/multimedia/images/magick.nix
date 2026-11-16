{
  flake.modules.homeManager.programs-multimedia-images-magick = {pkgs, ...}: {
    home.packages = with pkgs; [imagemagick];
  };
}
