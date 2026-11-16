{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    packages.mcfs = pkgs.callPackage ./package.nix {};

    devShells.gui = pkgs.mkShell {
      strictDeps = true;

      inputsFrom = [config.packages.mcfs];

      nativeBuildInputs = with pkgs;
        [
          cargoWrapper
          dbus
          libsoup_3
          nodejs_24
          openssl
          pkg-config
          rustToolchain
        ]
        ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
          webkitgtk_4_1
        ];
    };
  };
}
