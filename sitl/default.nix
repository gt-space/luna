{
  perSystem = {
    config,
    pkgs,
    lib,
    ...
  }:
    lib.mkIf pkgs.stdenv.hostPlatform.isLinux {
      legacyPackages =
        {
          sitl.isolab = pkgs.callPackage ./package.nix {};
        }
        // pkgs.callPackage ./apps.nix {};

      # callPackage adds two extra attrs to the set
      # perSystem.checks expects everything to be a derivation
      # thus...
      checks = builtins.removeAttrs (pkgs.callPackage ./checks.nix {
        servo = config.legacyPackages.servo.servo;
        flight = config.legacyPackages.flight2.flight-computer;
        harness = config.legacyPackages.sitl.isolab;
        common = config.legacyPackages.common.sequences;
      }) ["override" "overrideDerivation"];

      devShells.sitl = pkgs.mkShell {
        strictDeps = true;
      };
    };
}
