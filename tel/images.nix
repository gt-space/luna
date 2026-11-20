{inputs}: let
  inherit (inputs.nixpkgs) lib;

  # Each Nix file in these directories contributes a target variant.
  enumerateTargets = path:
    map (lib.removeSuffix ".nix") (
      builtins.attrNames (
        lib.filterAttrs
        (name: type: type == "regular" && lib.hasSuffix ".nix" name)
        (builtins.readDir path)
      )
    );

  targets = lib.cartesianProduct {
    build = enumerateTargets ./build;
    deployment = enumerateTargets ./deployment;
    platform = enumerateTargets ./platform;
  };

  mkImage = {
    build,
    deployment,
    platform,
  }: let
    nixosConfig = lib.nixosSystem {
      specialArgs = {inherit (inputs) nixos-hardware self sx1280;};

      modules = [
        ./build/${build}.nix
        ./deployment/${deployment}.nix
        ./platform/${platform}.nix
      ];
    };
  in
    lib.nameValuePair "${deployment}-${platform}-${build}" nixosConfig.config.system.build.sdImage;
in
  builtins.listToAttrs (map mkImage targets)
