{ crane, flake-utils, nixpkgs, nixos-generators, nixos-hardware, rust-overlay, sx1280, ... }:
let
  inherit (flake-utils.lib) mkApp;

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

  overlays = [ (import rust-overlay) ];
in
flake-utils.lib.eachDefaultSystem (system:
  let
    pkgs = import nixpkgs { inherit overlays system; };
    rust = pkgs.rust-bin.fromRustupToolchainFile ../rust-toolchain.toml;
    craneLib = (crane.mkLib pkgs).overrideToolchain (_: rust);

    proxy = craneLib.buildPackage {
      src = craneLib.cleanCargoSource ./.;
      strictDeps = true;
    };

    # proxy = craneLib.buildPackage {
    #   pname = "tel-proxy";
    #   version = "1.0.0";
    #
    #   src = pkgs.lib.cleanSourceWith {
    #     src = craneLib.path ../.;
    #
    #     filter = path: type:
    #       let
    #         base = baseNameOf path;
    #         pathStr = toString path;
    #         subProjStr = toString ./.;
    #       in
    #       (base == "Cargo.toml")
    #       || (base == "Cargo.lock")
    #       || (pkgs.lib.hasPrefix subProjStr pathStr);
    #   };
    #
    #   cargoExtraArgs = "-p tel";
    # };
  in
  rec {
    apps.tel.flash = builtins.mapAttrs (name: pkg: {
      debug = mkApp { drv = pkg.flasher.debug; };
      release = mkApp { drv = pkg.flasher.release; };
    }) packages;

    devShells.default = pkgs.mkShell {
      nativeBuildInputs = with pkgs; [
        rust
        rpiboot
      ];
    };

    packages.tel = {
      inherit proxy;
    } // (builtins.mapAttrs (name: path:
      mkDeploymentPackages {
        inherit pkgs;
        deployment = path;
        deploymentName = name;
      }
    ) deployments);
  }
)
