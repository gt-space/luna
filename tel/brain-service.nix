{config, ...}: {
  flake.overlays.tel-brain = final: prev: {
    tel = (prev.tel or {}) // {brain = final.callPackage ./brain.nix {};};
  };

  flake.nixosModules.tel-brain = {pkgs, ...}: {
    nixpkgs.overlays = [config.flake.overlays.tel-brain];

    systemd.services.tel-brain = {
      description = "Telemetry Brain";
      wantedBy = ["multi-user.target"];
      after = ["network.target"];

      serviceConfig = {
        Type = "simple";
        ExecStart = "${pkgs.tel.brain}/bin/tel-brain";
        Restart = "always";
        RestartSec = "1s";
      };
    };
  };
}
