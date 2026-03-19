{ ... }:
{
  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cargoWrapper
        rustNightly
        rustToolchain
      ];
    };
  };
}
