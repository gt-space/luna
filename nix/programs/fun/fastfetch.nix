{
  flake.modules.homeManager.programs-fun-fastfetch = {pkgs, ...}: {
    programs.fastfetch.enable = true;
  };
}
