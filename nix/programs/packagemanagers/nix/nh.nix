{
  flake.modules.nixos.programs-packagemanagers-nix = {
    programs.nh = {
      enable = true;

      clean = {
        enable = true;

        dates = "weekly";
        extraArgs = "--keep 3 --keep-since 30d --optimise --no-direnv";
      };
    };
  };
}
