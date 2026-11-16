{
  perSystem = {pkgs, ...}: {
    devShells.utility = pkgs.mkShell {
      strictDeps = true;

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
