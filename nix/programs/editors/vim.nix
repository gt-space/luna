{
  flake.modules.homeManager.programs-editors-vim = {pkgs, ...}: {
    home.packages = with pkgs; [vim];
  };
}
