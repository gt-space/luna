{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    legacyPackages.common.sequences = pkgs.callPackage ./package.nix {};

    devShells.common = pkgs.mkShell {
      strictDeps = true;
    };
  };
}
