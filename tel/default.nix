{ nixos-hardware, nixpkgs, self, sx1280, ... }:
let
  inherit (nixpkgs) lib;

  # Determine build, deployment, and platform targets dynamically by reading the
  # respective directories for entries.
  enumerateTargets = path:
    let
      entries = builtins.readDir path;
      nixFiles = builtins.attrNames (
        lib.filterAttrs (name: type:
          type == "regular" && lib.hasSuffix ".nix" name
        ) entries
      );
      filenames = map (name: lib.removeSuffix ".nix" name) nixFiles;
    in
    filenames;

  targets = lib.cartesianProduct {
    build = enumerateTargets ./build;
    deployment = enumerateTargets ./deployment;
    platform = enumerateTargets ./platform;
  };

  mkConfig = { platform, deployment, build, pkgs }: lib.nixosSystem {
    specialArgs = { inherit nixos-hardware self sx1280; };

    modules = [
      # Set cross-compilation platform.
      # { nixpkgs.buildPlatform = pkgs.stdenv.hostPlatform.system; }

      ./build/${build}.nix
      ./deployment/${deployment}.nix
      ./platform/${platform}.nix
    ];
  };

  mkFlasher = { image, pkgs, target }:
  let
    # Path to the image to flash.
    path = "\"${image}/sd-image/\"nixos-*.img";

    # Darwin hosts do not support bmaptool, so they must use dd instead.
    darwinFlash = ''
      ${pkgs.coreutils}/bin/dd \
        if=${path} \
        of="$DEVICE" \
        status=progress \
        conv=fsync
    '';

    linuxFlash = ''
      ${pkgs.bmaptool}/bin/bmaptool copy --nobmap ${path} "$DEVICE"
    '';
  in
  pkgs.writeShellScriptBin "${target}-flasher" ''
    #!${pkgs.bash}/bin/bash
    set -e

    DEVICE="$1"

    # Check that a device was specified.
    if [ -z "$DEVICE" ]; then
      echo "error: no device path specified" >&2
      echo "usage: nix run .#tel.flash.<flight|ground>-<cm4|4b>-<debug|release> -- /dev/sdX" >&2
      exit 1
    fi

    echo "Flashing ${path} to $DEVICE..."
    ${if pkgs.stdenv.isDarwin then darwinFlash else linuxFlash}
  '';
in
{
  apps = pkgs:
    let
      inherit (pkgs.stdenv.hostPlatform) system;
    in
    {
      tel = {
        brain = {
          type = "app";
          program = "${self.packages.${system}.tel.brain}/bin/tel-brain";
        };

        flash = builtins.mapAttrs
          (target: flasher: {
            type = "app";
            program = "${flasher}/bin/${target}-flasher";
          })
          self.packages.${system}.tel.flashers;
      };
    };

  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        rpiboot
        rustToolchain
      ];
    };
  };

  nixosModules.tel.brain = { pkgs, ... }: {
    nixpkgs.overlays = [ self.overlays.tel.brain ];
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

  overlays.tel.brain = final: prev: {
    tel.brain = self.packages.${prev.stdenv.hostPlatform.system}.tel.brain;
  };

  packages = pkgs:
    let
      brain = pkgs.rustPlatform.buildRustPackage {
        pname = "tel-brain";
        version = "1.0.0";
        src = ./brain;
        cargoLock.lockFile = ./brain/Cargo.lock;
      };

      images = builtins.listToAttrs (map (target:
        let
          nixosConfig = mkConfig (target // { inherit pkgs; });
        in
        {
          name = "${target.deployment}-${target.platform}-${target.build}";
          value = nixosConfig.config.system.build.sdImage;
        }
      ) targets);

      flashers = builtins.mapAttrs
        (target: image: mkFlasher { inherit image pkgs target; })
        images;
    in
    {
      tel = { inherit brain flashers images; };
    };
}
