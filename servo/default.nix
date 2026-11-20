{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    legacyPackages.servo.servo = pkgs.callPackage ./package.nix {};

    devShells.servo = pkgs.mkShell {
      strictDeps = true;

      buildInputs = with pkgs; [
        openssl
      ];

      nativeBuildInputs = with pkgs; [
        cmake
        cargoWrapper
        hdf5
        pkg-config
        rustToolchain
      ];
    };
  };
}
