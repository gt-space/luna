{
  flake.modules.homeManager.programs-multimedia-videos-ffmpeg = {pkgs, ...}: {
    home.packages = with pkgs; [ffmpeg];
  };
}
