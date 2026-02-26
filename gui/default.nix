{ ... }:
{
  devShells.default = pkgs: pkgs.mkShell {
    packages = with pkgs; [
      dbus
      libsoup_3
      nodejs_24
      openssl
      pkg-config
      rustToolchain
    ] ++ pkgs.lib.optional pkgs.stdenv.isLinux [
      webkitgtk_4_1
    ];
  };
}
