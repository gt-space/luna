{rustPlatform}:
rustPlatform.buildRustPackage {
  pname = "tel-simulator";
  version = "1.0.0";
  src = ./.;
  cargoLock.lockFile = ./Cargo.lock;
}
