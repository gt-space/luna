{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-hardware.url = "github:NixOS/nixos-hardware";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    sx1280 = {
      url = "github:jeffcshelton/sx1280";
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
  let
    inherit (flake-utils.lib) mkApp;

    # Auto-discover deployment configurations
    deployments = {
      devkit = ./deployments/devkit.nix;
      flight = ./deployments/flight.nix;
      ground = ./deployments/ground.nix;
    };

    mkFlasher = { pkgs, image, name }:
    pkgs.writeShellScriptBin "${name}-flasher" ''
      #!${pkgs.bash}/bin/bash
      set -e

      DEVICE="$1"

      # Check that a device was specified.
      if [ -z "$DEVICE" ]; then
        echo "error: no device path specified" >&2
        echo "usage: nix run .#flash.<deployment>.<debug|release> -- /dev/sdX" >&2
        exit 1
      fi

      echo "Flashing ${name} image to $DEVICE..."
      ${pkgs.bmaptool}/bin/bmaptool copy --nobmap "${image}" "$DEVICE"
    '';

    # Helper function to generate packages for a specific deployment
    mkDeploymentPackages = { pkgs, deployment, deploymentName }:
      let
        debugName = "tel-${deploymentName}-debug";
        releaseName = "tel-${deploymentName}-release";
        version = "1.0.0-dev";

        releaseCompressed = nixos-generators.nixosGenerate {
          format = "sd-aarch64";
          modules = [ ./release.nix deployment ];
          specialArgs = { inherit nixos-hardware sx1280 version; };
          system = "aarch64-linux";
        };

        debugCompressed = nixos-generators.nixosGenerate {
          format = "sd-aarch64";
          modules = [ ./debug.nix deployment ];
          specialArgs = { inherit nixos-hardware sx1280 version; };
          system = "aarch64-linux";
        };
      in
      rec {
        debug = pkgs.runCommand "${debugName}.img" {} ''
          ${pkgs.zstd}/bin/zstd \
            -d ${debugCompressed}/sd-image/nixos-image-*.img.zst \
            -o $out
        '';

        release = pkgs.runCommand "${releaseName}.img" {} ''
          ${pkgs.zstd}/bin/zstd \
            -d ${releaseCompressed}/sd-image/nixos-image-*.img.zst \
            -o $out
        '';

        flasher = {
          release = mkFlasher {
            inherit pkgs;
            image = release;
            name = releaseName;
          };

          debug = mkFlasher {
            inherit pkgs;
            image = debug;
            name = debugName;
          };
        };
      };
  in
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs { inherit system; };

      # Generate packages for all deployments
      deploymentPackages = builtins.mapAttrs (name: path:
        mkDeploymentPackages {
          inherit pkgs;
          deployment = path;
          deploymentName = name;
        }
      ) deployments;

      # Generate flash apps for all deployments
      flashApps = builtins.mapAttrs (name: pkg: {
        release = mkApp { drv = pkg.flasher.release; };
        debug = mkApp { drv = pkg.flasher.debug; };
      }) deploymentPackages;
    in
    {
      packages = deploymentPackages;
      apps.flash = flashApps;
    }
  );
}
