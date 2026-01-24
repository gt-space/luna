{ flake-utils, nixpkgs, ... }:
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs { inherit system; };
  in
  {
    devShells.default = pkgs.mkShell {
      RENODE_PATH = pkgs.renode-bin;

      nativeBuildInputs = with pkgs; [
        dotnet-sdk_8
        renode-bin
      ];
    };
  }
)
