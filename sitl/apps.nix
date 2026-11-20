{
  stdenv,
  writeShellScriptBin,
  nix,
}: let
  inherit (stdenv.hostPlatform) system;
  mkRunApp = name: check:
    writeShellScriptBin "isolab-${name}" ''
      check_ref="${builtins.toString ../.}#checks.${system}.${check}"
      exec ${nix}/bin/nix build "$check_ref" -L --no-link "$@"
    '';
in {
  default-source-umbilical = mkRunApp "default-source-umbilical" "default-source-umbilical";
  radio-survives-disconnect = mkRunApp "radio-survives-disconnect" "radio-survives-disconnect";
  vespula-radio-forwarding = mkRunApp "vespula-radio-forwarding" "vespula-radio-forwarding";
  radio-without-sam = mkRunApp "radio-without-sam" "radio-without-sam";
}
