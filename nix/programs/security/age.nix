{
  flake.modules.homeManager.programs-security-age = {pkgs, ...}: {
    home.packages = with pkgs; [age];
  };
}
