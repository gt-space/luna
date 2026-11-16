{
  flake.modules.homeManager.programs-monitoring-nvtop = {pkgs, ...}: {
    home.packages = with pkgs; [nvtopPackages.full];
  };
}
