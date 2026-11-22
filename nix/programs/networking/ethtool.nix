{
  flake.modules.homeManager.programs-networking-ethtool = {pkgs, ...}: {
    home.packages = with pkgs; [ethtool];
  };
}
