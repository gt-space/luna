{ pkgs, ... }:
let
  renode-lib = pkgs.stdenv.mkDerivation rec {
    pname = "renode-lib";
    version = "1.16.0";

    src = pkgs.fetchurl {
      url = "https://github.com/renode/renode/releases/download/v${version}/renode-${version}.linux-dotnet.tar.gz";
      sha256 = "sha256-oNlTz5LBggPkjKM4TJO2UDKQdt2Ga7rBTdgyGjN8/zA=";
    };

    dontBuild = true;
    installPhase = ''
      mkdir -p $out/lib
      cp bin/*.dll $out/lib/
    '';
  };
in
{
  devShells.default = pkgs.mkShell {
    RENODE_PATH = renode-lib;

    packages = with pkgs; [
      dotnet-sdk_8
      renode-lib
    ];
  };
}
