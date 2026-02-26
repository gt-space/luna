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
    # Auto-discover all subprojects.
    subprojectNames = builtins.filter
      (name: builtins.pathExists ./${name}/default.nix)
      (builtins.attrNames (builtins.readDir ./.));

    # Evaluate subproject functions on flake inputs (name → outputs attrset).
    subprojectOutputs = builtins.listToAttrs
      (map
        (name: { inherit name; value = import ./${name} inputs; })
        subprojectNames);

    # Segment out top-level attributes that are not system-dependent.
    top-level = builtins.removeAttrs
      subprojectOutputs
      [ "apps" "devShells" "packages" ];
  in
  flake-utils.lib.eachDefaultSystem (system:
    let
      inherit (nixpkgs) lib;

      # Construct the global pkgs object, merging in all necessary overlays.
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

      # Call and merge system-dependent attributes from subprojects.
      merge = attr: lib.concatMapAttrs
        (name: subproject:
          if builtins.hasAttr attr subproject
          then
            let val = subproject.${attr}; in
            if builtins.isFunction val
            then val pkgs
            else { ${name} = builtins.mapAttrs (_: f: f pkgs) val; }
          else {}
        )
        subprojectOutputs;

      apps = merge "apps";
      devShells = merge "devShells";
      packages = merge "packages";

      # Gather all default subshells from subprojects.
      subshells = lib.pipe (builtins.attrValues devShells) [
        (builtins.filter (shells: shells ? default))
        (map (shells: shells.default))
      ];

      subshellEnvVars = lib.foldl' (acc: subshell:
        acc // (lib.filterAttrs
          (name: _: builtins.match "[A-Z_][A-Z0-9_]*" name != null)
          subshell
        )
      ) {} subshells;

      subshellHooks = lib.concatStringsSep "\n"
        (map (shell: shell.shellHook or "") subshells);
    in
    { inherit apps devShells packages; }
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
