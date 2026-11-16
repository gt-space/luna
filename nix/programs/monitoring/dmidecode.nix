{
  flake.modules.homeManager.programs-monitoring-dmidecode = {pkgs, ...}: {
    home.packages = with pkgs; [dmidecode];
  };
}
