{
  flake-utils,
  nixpkgs,
  nixos-generators,
  nixos-hardware,
  sx1280,
  ...
} @ inputs:
let
  inherit (flake-utils.lib) mkApp;
  inherit (nixpkgs) lib;

  brain = import ./brain inputs;

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

  mkImage = { pkgs, platform, deployment, build }:
  let
    compressed = nixos-generators.nixosGenerate {
      format = "sd-aarch64";
      specialArgs = { inherit brain nixos-hardware sx1280; };
      system = "aarch64-linux";

      modules = [
        ./build/${build}.nix
        ./deployment/${deployment}.nix
        ./platform/${platform}.nix
      ];
    };
  in
  pkgs.runCommand "tel-${deployment}-${platform}-${build}.img" {} ''
    ${pkgs.zstd}/bin/zstd \
      -d ${compressed}/sd-image/nixos-image-*.img.zst \
      -o $out
  '';

  mkFlasher = { pkgs, image }:
  let
    # Darwin hosts do not support bmaptool, so they must use dd instead.
    darwinFlash = ''
      ${pkgs.coreutils}/bin/dd if="${image}" of="$DEVICE" status=progress conv=fsync
    '';

    linuxFlash = ''
      ${pkgs.bmaptool}/bin/bmaptool copy --nobmap "${image}" "$DEVICE"
    '';
  in
  pkgs.writeShellScriptBin "${image.name}-flasher" ''
    #!${pkgs.bash}/bin/bash
    set -e

    DEVICE="$1"

    # Check that a device was specified.
    if [ -z "$DEVICE" ]; then
      echo "error: no device path specified" >&2
      echo "usage: nix run .#flash.<deployment>.<debug|release> -- /dev/sdX" >&2
      exit 1
    fi

    echo "Flashing ${image} to $DEVICE..."
    ${if pkgs.stdenv.isDarwin then darwinFlash else linuxFlash}
  '';
in
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs {
      inherit system;
      overlays = [ brain.overlays.default ];
    };

    # Construct an image derivation for every target.
    images = builtins.listToAttrs (map (target: {
      name = "${target.deployment}-${target.platform}-${target.build}";
      value = mkImage {
        inherit (target) build deployment platform;
        inherit pkgs;
      };
    }) targets);
  in
  {
    apps.tel.flash = builtins.mapAttrs (name: image:
      let
        flasher = mkFlasher { inherit pkgs image; };
      in
      mkApp { drv = flasher; }
    ) images;

    devShells.default = pkgs.mkShell {
      nativeBuildInputs = with pkgs; [
        cargo
        rpiboot
        rust-analyzer
        rustc
        rustfmt
      ];
    };

    packages.tel = {
      inherit (pkgs.tel) brain;
    } // images;
  }
)
