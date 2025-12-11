{ flake-utils, nixpkgs, ... }:
let
  inherit (flake-utils) mkApp;

  overlay = final: prev: {
    tel.brain = final.rustPlatform.buildRustPackage {
      pname = "tel-brain";
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
    apps.default = mkApp { drv = pkgs.tel.brain; };
    packages.default = pkgs.tel.brain;
  }
) // {
  nixosModules.default = { pkgs, ... }: {
    nixpkgs.overlays = [ overlay ];
    systemd.services.tel-brain = {
      description = "Telemetry Brain";
      wantedBy = [ "multi-user.target" ];
      after = [ "network.target" ];

      serviceConfig = {
        Type = "simple";
        ExecStart = "${pkgs.tel.brain}/bin/tel-brain";
        Restart = "always";
        RestartSec = "1s";
      };
    };
  };

  overlays.default = overlay;
}
