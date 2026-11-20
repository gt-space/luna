{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    legacyPackages.gui.mcfs = pkgs.callPackage ./package.nix {};

    devShells.gui = pkgs.mkShell {
      strictDeps = true;

      buildInputs = with pkgs;
        [
          openssl
        ]
        ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
          webkitgtk_4_1
        ];

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        dbus
        libsoup_3
        nodejs_24
        pkg-config
        rustToolchain
      ];
    };
  };
}
