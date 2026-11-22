{
  flake.modules.homeManager.programs-networking-nmap = {pkgs, ...}: {
    home.packages = with pkgs; [nmap];
  };
}
