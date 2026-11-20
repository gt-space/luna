{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    legacyPackages.flight2.flight-computer = pkgs.callPackage ./package.nix {};

    devShells.flight2 = pkgs.mkShell {
      strictDeps = true;

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
