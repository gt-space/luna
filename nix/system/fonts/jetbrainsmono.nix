{
  flake.modules.homeManager.system-fonts-jetbrainsmono = {pkgs, ...}: {
    home.packages = with pkgs; [nerd-fonts.jetbrains-mono];
  };
}
