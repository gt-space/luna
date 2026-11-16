{
  flake.modules.homeManager.system-fonts-microsoftfonts = {pkgs, ...}: {
    home.packages = with pkgs; [
      corefonts
      vista-fonts
    ];
  };
}
