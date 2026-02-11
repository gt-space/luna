{ pkgs, ... }:
{
  devShells.default = pkgs.mkShell {
    packages = with pkgs; [
      cmake
      hdf5
      openssl
      pkg-config
      rustToolchain
    ];
  };
}
