{ ... }:
{
  devShells.default = pkgs: pkgs.mkShell {
    packages = with pkgs; [
      cmake
      hdf5
      openssl
      pkg-config
      rustToolchain
    ];
  };
}
