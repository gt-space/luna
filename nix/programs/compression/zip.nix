{
  flake.modules.homeManager.programs-compression-zip = {pkgs, ...}: {
    home.packages = with pkgs; [
      zip
      unzip
    ];
  };
}
