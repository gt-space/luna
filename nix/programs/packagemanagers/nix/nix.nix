{self, ...}: {
  flake.modules.nixos.programs-packagemanagers-nix = {pkgs, ...}: {
    documentation.nixos.enable = false;

    nix = {
      package = pkgs.lixPackageSets.stable.lix;

      settings.experimental-features = [
        "nix-command"
        "flakes"
      ];
    };

    nixpkgs = {
      config.allowUnfree = true;
      overlays = [self.overlays.default];
    };
  };
}
