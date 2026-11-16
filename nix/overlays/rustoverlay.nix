{inputs, ...}: {
  perSystem = {system, ...}: {
    _module.args.pkgs = import inputs.nixpkgs {
      inherit system;
      overlays = [
        (import inputs.rust-overlay)
        (final: prev: {
          rustStable = prev.rust-bin.stable.latest.default.override {
            extensions = ["rust-analyzer" "rust-src"];
            targets = [
              "armv7-unknown-linux-musleabihf"
              "x86_64-unknown-linux-gnu"
            ];
          };

          rustNightly = prev.rust-bin.selectLatestNightlyWith (toolchain:
            toolchain.minimal.override {
              extensions = ["rustfmt"];
            });

          # Nix toolchains do not come with rustup's `cargo +toolchain`
          # selector behavior, so this wrapper preserves `cargo +nightly`
          # specifically for nightly rustfmt while keeping stable cargo the
          # default for all other commands.
          cargoWrapper = prev.writeShellScriptBin "cargo" ''
            if [ "''${1:-}" = "+nightly" ]; then
              shift
              exec ${final.rustNightly}/bin/cargo "$@"
            fi

            exec ${final.rustStable}/bin/cargo "$@"
          '';

          rustToolchain = final.rustStable;
        })
      ];

      config = {};
    };
  };
}
