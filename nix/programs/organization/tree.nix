{
  flake.modules.homeManager.programs-organization-tree = {pkgs, ...}: {
    home.packages = with pkgs; [tree];
  };
}
