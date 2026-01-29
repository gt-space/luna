{
  description = "YJSP Developer Shell and Build Environments";

  inputs = {
    crane.url = "github:ipetkov/crane";
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { nixpkgs, ... } @ inputs:
  let
    inherit (nixpkgs) lib;

    projectPaths = [
      ./sam
    ];

    projectOutputs = map (path: import path inputs) projectPaths;
  in
  lib.foldr lib.recursiveUpdate { } projectOutputs;
}
