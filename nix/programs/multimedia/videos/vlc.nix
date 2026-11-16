{
  flake.modules.homeManager.programs-multimedia-videos-vlc = {pkgs, ...}: {
    home.packages = with pkgs; [vlc];
  };
}
