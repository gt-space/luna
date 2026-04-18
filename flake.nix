{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";

    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    nixos-hardware.url = "github:NixOS/nixos-hardware";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    sx1280.url = "github:jeffcshelton/sx1280";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { flake-utils, nixpkgs, rust-overlay, ... } @ inputs:
  let
    inherit (nixpkgs) lib;

    # Auto-discover all subprojects.
    subprojectNames = builtins.filter
      (name: builtins.pathExists ./${name}/default.nix)
      (builtins.attrNames (builtins.readDir ./.));

    # Evaluate subproject functions on flake inputs.
    subprojectOutputs = map
      (name: import ./${name} inputs)
      subprojectNames;

    # Segment out top-level attributes that are not system-dependent.
    top-level = lib.foldl lib.recursiveUpdate {}
      (map
        (attrs: builtins.removeAttrs
          attrs
          [ "apps" "checks" "devShells" "packages" ]
        )
        subprojectOutputs
      );
  in
  flake-utils.lib.eachDefaultSystem (system:
    let
      # Construct the global pkgs object, merging in all necessary overlays.
      pkgs = import nixpkgs {
        inherit system;
        overlays = [
          (import rust-overlay)
          (final: prev: {
            rustStable = prev.rust-bin.stable.latest.default.override {
              extensions = [ "rust-analyzer" "rust-src" ];
              targets = [
                "armv7-unknown-linux-musleabihf"
                "x86_64-unknown-linux-gnu"
              ];
            };

            rustNightly =
              prev.rust-bin.selectLatestNightlyWith (toolchain:
                toolchain.minimal.override {
                  extensions = [ "rustfmt" ];
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
      };

      # Call and merge system-dependent attributes from subprojects.
      merge = attr: lib.foldl lib.recursiveUpdate {}
        (map
          (subproject:
            if subproject ? ${attr}
            then subproject.${attr} pkgs
            else {}
          )
          subprojectOutputs
        );

      apps = merge "apps";
      checks = merge "checks";
      devShells = merge "devShells";
      packages = merge "packages";

      # Gather all default shells from subprojects and merge them.
      subshells = builtins.concatMap
        (subproject:
          let
            defaultShell = if subproject ? devShells
              then (subproject.devShells pkgs).default or null
              else null;
          in
          if defaultShell != null
            then [ defaultShell ]
            else [ ]
        )
        subprojectOutputs;

      subshellEnvVars = lib.foldl' (acc: subshell:
        acc // (lib.filterAttrs
          (name: _: builtins.match "[A-Z_][A-Z0-9_]*" name != null)
          subshell
        )
      ) {} subshells;

      subshellHooks = lib.concatStringsSep "\n"
        (map (shell: shell.shellHook or "") subshells);
    in
    { inherit apps checks devShells packages; }
    // {
      devShells.default = pkgs.mkShell (
        {
          inputsFrom = subshells;
          shellHook = subshellHooks;
        }
        // subshellEnvVars
      );
    }
  ) // top-level;
}
