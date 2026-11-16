{
  rustPlatform,
  lib,
  cmake,
  pkg-config,
  python3,
  openssl,
}:
rustPlatform.buildRustPackage {
  pname = "servo";
  version = "0.1.0";
  src = ../.;

  cargoLock = {
    lockFile = ../Cargo.lock;
    outputHashes = {
      "hdf5-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
      "hdf5-derive-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
      "hdf5-src-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
      "hdf5-sys-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
      "hdf5-types-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
    };
  };

  doCheck = false;

  cargoBuildFlags = [
    "--package"
    "servo"
    "--bin"
    "servo"
  ];
  nativeBuildInputs = [
    cmake
    pkg-config
    python3
  ];
  buildInputs = [
    openssl
  ];
}
