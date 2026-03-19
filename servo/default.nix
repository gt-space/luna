{ ... }:
{
  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cmake
        cargoWrapper
        hdf5
        openssl
        pkg-config
        rustNightly
        rustToolchain
      ];
    };
  };
}
