{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-hardware.url = "github:NixOS/nixos-hardware";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    sx1280 = {
      url = "path:/home/jeff/Dev/sx1280"; # TODO: change to github
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    flake-utils,
    nixpkgs,
    nixos-generators,
    nixos-hardware,
    sx1280,
    ...
  }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs { inherit system; };
      version = "1.0.0-dev";
    in
    rec {
      apps.flash = {
        type = "app";
        program = "${packages.flasher}/bin/flasher";
      };

      packages = rec {
        flasher = pkgs.writeShellScriptBin "flasher" ''
          #!${pkgs.bash}/bin/bash
          set -e

          DEVICE="$1"

          # Check that a device was specified.
          if [ -z "$DEVICE" ]; then
            echo "error: no device path specified" >&2
            echo "usage: nix run .#tel.flash -- /dev/sdX" >&2
            exit 1
          fi

          ${pkgs.bmaptool}/bin/bmaptool copy --nobmap "${debug}" "$DEVICE"
        '';

        releaseCompressed = nixos-generators.nixosGenerate {
          format = "sd-aarch64";
          modules = [ ./release.nix ];
          specialArgs = { inherit nixos-hardware sx1280 version; };
          system = "aarch64-linux";
        };

        debugCompressed = nixos-generators.nixosGenerate {
          format = "sd-aarch64";
          modules = [ ./debug.nix ];
          specialArgs = { inherit nixos-hardware sx1280 version; };
          system = "aarch64-linux";
        };

        debug = pkgs.runCommand "tel-debug.img" {} ''
          ${pkgs.zstd}/bin/zstd \
            -d ${debugCompressed}/sd-image/nixos-image-*.img.zst \
            -o $out
        '';

        release = pkgs.runCommand "tel-release.img" {} ''
          ${pkgs.zstd}/bin/zstd \
            -d ${releaseCompressed}/sd-image/nixos-image-*.img.zst \
            -o $out
        '';
      };
    }
  );
}
