{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    devShells.sam = pkgs.mkShell {
      strictDeps = true;

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
