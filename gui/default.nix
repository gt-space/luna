{ lib, pkgs, ... }:
{
  devShells.default = pkgs.mkShell {
    packages = with pkgs; [
      dbus
      libsoup_3
      nodejs_24
      openssl
      pkg-config
      rust
    ] ++ lib.optional pkgs.stdenv.isLinux [
      webkitgtk_4_1
    ];
  };
}
