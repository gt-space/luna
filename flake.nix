{
  description = "YJSP Developer Shell and Build Environments (feat. flake-parts)";

  inputs = {
    flake-compat.url = "github:nixos/flake-compat";
    flake-parts.url = "github:hercules-ci/flake-parts";
    nix-index-database.url = "github:nix-community/nix-index-database";

    nixpkgs.url = "github:nixos/nixpkgs/nixos-26.05";

    catppuccin = {
      url = "github:catppuccin/nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    home-manager = {
      url = "github:nix-community/home-manager/release-26.05";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    nix-alien = {
      url = "github:thiagokokada/nix-alien";

      inputs = {
        flake-compat.follows = "flake-compat";
        nix-index-database.follows = "nix-index-database";
        nixpkgs.follows = "nix-index-database/nixpkgs";
      };
    };

    nixos-hardware = {
      url = "github:nixos/nixos-hardware";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    sx1280.url = "github:jeffcshelton/sx1280";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs: let
    inherit (inputs.nixpkgs) lib;
    isModule = file: file.hasExt "nix" && file.name != "flake.nix";
    modulesAt = path: lib.fileset.toList (lib.fileset.fileFilter isModule path);
  in
    inputs.flake-parts.lib.mkFlake {inherit inputs;} {
      imports =
        modulesAt ./nix
        ++ [
          ./bms
          ./common
          ./flight2
          ./gui
          ./sam
          ./servo
          ./utility
        ];
    };
}
