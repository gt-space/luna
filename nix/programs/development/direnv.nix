{
  flake.modules.homeManager.programs-development-direnv = {
    programs.direnv = {
      enable = true;
      nix-direnv.enable = true;
    };
  };
}
