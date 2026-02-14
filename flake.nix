{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { flake-utils, nixpkgs, rust-overlay, ... }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      inherit (nixpkgs) lib;

      pkgs = import nixpkgs {
        inherit system;
        overlays = [
          (import rust-overlay)
          (final: prev: {
            rustToolchain = prev.rust-bin.stable.latest.default.override {
              extensions = [ "rust-analyzer" ];
              targets = [
                "armv7-unknown-linux-musleabihf"
                "x86_64-unknown-linux-gnu"
              ];
            };
          })
        ];
      };

      # Auto-discover all subprojects.
      subprojects = map
        (name: ./${name})
        (builtins.filter
          (name: builtins.pathExists ./${name}/default.nix)
          (builtins.attrNames (builtins.readDir ./.))
        );

      # Arguments passed to subproject functions.
      args = {
        inherit lib pkgs system;
      };

      # Evaluate subproject functions on flake inputs.
      subprojectOutputs = map
        (path: import path args)
        subprojects;

      # Gather all default subshells from subprojects.
      subshells = builtins.filter
        (sh: sh != null)
        (map
          (output:
            if (output ? devShells.default)
            then output.devShells.default
            else null
          )
          subprojectOutputs
        );

      subshellEnvVars = lib.foldl' (acc: subshell:
        acc // (lib.filterAttrs
          (name: _: builtins.match "[A-Z_][A-Z0-9_]*" name != null)
          subshell
        )
      ) {} subshells;

      subshellHooks = lib.concatStringsSep "\n"
        (map (shell: shell.shellHook or "") subshells);
    in

    # Merge all attributes from subprojects.
    (lib.foldr lib.recursiveUpdate { } subprojectOutputs)

    # Merge all default dev-shells.
    // {
      devShells.default = pkgs.mkShell (
        {
          inputsFrom = subshells;
          shellHook = subshellHooks;
        }
        // subshellEnvVars
      );
    }
  );
}
