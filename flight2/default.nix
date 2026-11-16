{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    packages.flight-computer = pkgs.callPackage ./package.nix {};

    devShells.flight2 = pkgs.mkShell {
      strictDeps = true;

      inputsFrom = [config.packages.flight-computer];

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
