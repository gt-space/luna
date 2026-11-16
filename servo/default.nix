{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    packages.servo = pkgs.callPackage ./package.nix {};

    devShells.servo = pkgs.mkShell {
      strictDeps = true;

      inputsFrom = [config.packages.servo];

      nativeBuildInputs = with pkgs; [
        cmake
        cargoWrapper
        hdf5
        openssl
        pkg-config
        rustToolchain
      ];
    };
  };
}
