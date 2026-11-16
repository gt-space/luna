{
  flake.modules.homeManager.programs-monitoring-smartctl = {pkgs, ...}: {
    home.packages = with pkgs; [smartmontools];
  };
}
