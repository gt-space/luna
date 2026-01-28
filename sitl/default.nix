{ flake-utils, nixpkgs, ... }:
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs { inherit system; };

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
      RENODE_PATH = pkgs.renode-bin;

      nativeBuildInputs = with pkgs; [
        linuxPkgs.renode-bin
        dotnet-sdk_8
      ];

      buildInputs = with pkgs; [ renode-bin ];
    };

    packages.sitl.peripherals = peripherals;
  }
)
