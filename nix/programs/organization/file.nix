{
  flake.modules.homeManager.programs-organization-file = {pkgs, ...}: {
    home.packages = with pkgs; [file];
  };
}
