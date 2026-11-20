{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    legacyPackages.servo.servo = pkgs.callPackage ./package.nix {};

    devShells.servo = pkgs.mkShell {
      strictDeps = true;

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
