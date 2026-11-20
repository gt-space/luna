{rustPlatform}:
rustPlatform.buildRustPackage {
  pname = "tel-brain";
  version = "1.0.0";
  src = ./brain;
  cargoLock.lockFile = ./brain/Cargo.lock;
}
