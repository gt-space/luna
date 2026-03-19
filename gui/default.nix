{ ... }:
{
  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cargoWrapper
        dbus
        libsoup_3
        nodejs_24
        openssl
        pkg-config
        rustNightly
        rustToolchain
      ] ++ pkgs.lib.optional pkgs.stdenv.isLinux [
        webkitgtk_4_1
      ];
    };
  };
}
