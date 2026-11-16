{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    packages.common-sequences = pkgs.callPackage ./package.nix {};

    devShells.common = pkgs.mkShell {
      strictDeps = true;
      inputsFrom = [config.packages.common-sequences];
    };
  };
}
