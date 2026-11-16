{
  flake.modules.homeManager.programs-organization-trash = {pkgs, ...}: {
    home.packages = with pkgs; [trash-cli];
  };
}
