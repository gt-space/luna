{
  flake.modules.homeManager.programs-math-gnomecalculator = {pkgs, ...}: {
    home.packages = with pkgs; [gnome-calculator];
  };
}
