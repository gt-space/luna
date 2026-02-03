{ flake-utils, nixpkgs, ... }:
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs {
      inherit system;
      config.allowUnsupportedSystem = true;
    };

    peripherals = pkgs.stdenv.mkDerivation {
      pname = "SITL-Peripheral";
      version = "0.1.0";
      src = ./.;

      nativeBuildInputs = with pkgs; [
        dotnet-sdk_8
      ];

      RENODE_PATH = pkgs.renode-bin;

      buildPhase = ''
        dotnet build -c Release
      '';

      installPhase = ''
        mkdir -p $out/lib
        cp bin/Release/net8.0/SITL.dll $out/lib/
      '';
    };

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

    monitor = pkgs.writeShellApplication {
      name = "sitl-monitor";
      runtimeInputs = with pkgs; [ renode-bin ];
      text = ''
        export SITL_DLL_PATH="${peripherals}/lib/SITL.dll"
      '';
    };
  in
  {
    apps.sitl.monitor = flake-utils.lib.mkApp { drv = monitor; };

    devShells.default = pkgs.mkShell {
      RENODE_PATH = renode-lib;

      nativeBuildInputs = with pkgs; [
        renode-lib
        dotnet-sdk_8
      ];

      shellHook = ''
        export RENODE_PATH=${renode-lib}
      '';
    };

    packages.sitl.renode = renode-lib;

    packages.sitl.peripherals = peripherals;
  }
)
