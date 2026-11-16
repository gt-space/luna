{
  flake.modules.homeManager.programs-utilities-tldr = {pkgs, ...}: {
    home.packages = with pkgs; [tlrc];

    services.tldr-update = {
      enable = true;
      package = pkgs.tlrc;
    };
  };
}
