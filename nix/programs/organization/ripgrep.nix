{
  flake.modules.homeManager.programs-organization-ripgrep = {pkgs, ...}: {
    programs.ripgrep.enable = true;
  };
}
