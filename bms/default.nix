{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    devShells.bms = pkgs.mkShell {
      strictDeps = true;

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
