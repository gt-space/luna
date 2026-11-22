{
  flake.modules.homeManager.programs-networking-netcat = {pkgs, ...}: {
    home.packages = with pkgs; [netcat-openbsd];
  };
}
