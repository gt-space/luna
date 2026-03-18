{ flake-utils, nixpkgs, ... }:
let
  inherit (flake-utils) mkApp;

  overlay = final: prev: {
    tel.simulator = final.rustPlatform.buildRustPackage {
      pname = "tel-simulator";
      version = "1.0.0";
      src = ./.;
      cargoLock.lockFile = ./Cargo.lock;
    };
  };
in
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs {
      inherit system;
      overlays = [ overlay ];
    };
  in
  {
    apps.default = mkApp { drv = pkgs.tel.simulator; };
    packages.default = pkgs.tel.simulator;
  }
) // {
  overlays.default = overlay;
}
