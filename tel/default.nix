{
  inputs,
  lib,
  ...
}: let
  images = import ./images.nix {inherit inputs;};
in {
  imports = [./brain-service.nix];

  perSystem = {pkgs, ...}: let
    brain = pkgs.callPackage ./brain.nix {};
    simulator = pkgs.callPackage ./simulator {};
    flashers =
      builtins.mapAttrs
      (target: image: pkgs.callPackage ./flasher.nix {inherit image target;})
      images;
  in {
    legacyPackages.tel = {
      inherit brain simulator images flashers;

      # Keep the existing nix run .#tel.flash.<target> commands.
      flash = flashers;
    };

    apps =
      {
        tel-brain.program = "${brain}/bin/tel-brain";
        tel-simulator.program = "${simulator}/bin/tel-simulator";
      }
      // lib.mapAttrs' (
        target: flasher: lib.nameValuePair "tel-flash-${target}" {program = flasher;}
      )
      flashers;

    devShells.tel = pkgs.mkShell {
      strictDeps = true;

      nativeBuildInputs = with pkgs; [
        cargoWrapper
        rpiboot
        rustToolchain
      ];
    };
  };
}
