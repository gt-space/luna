{ flake-utils, nixpkgs, ... }:
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs {
      inherit system;
      config.allowUnsupportedSystem = true;
    };

    linuxPkgs = import nixpkgs {
      system = "x86_64-linux";
      config.allowUnsupportedSystem = true;
    };
  in
  {
    devShells.default = pkgs.mkShell {
      RENODE_PATH = linuxPkgs.renode-bin;

      nativeBuildInputs = with pkgs; [
        linuxPkgs.renode-bin
        dotnet-sdk_8
      ];

      buildInputs = [ linuxPkgs.renode-bin ];
    };
  }
)
