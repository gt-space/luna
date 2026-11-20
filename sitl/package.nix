{
  rustPlatform,
  pkg-config,
  openssl,
}:
rustPlatform.buildRustPackage {
  pname = "isolab";
  version = "0.1.0";

  src = builtins.path {
    path = ../.;
    name = "luna-source";
  };

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
    "isolab"
    "--bin"
    "isolab"
  ];

  nativeBuildInputs = [
    pkg-config
  ];

  buildInputs = [
    openssl
  ];
}
