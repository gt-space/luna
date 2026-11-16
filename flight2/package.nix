{
  rustPlatform,
  makeWrapper,
  pkg-config,
  python3,
}:
rustPlatform.buildRustPackage {
  pname = "flight-computer";
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
    "flight-computer"
    "--bin"
    "flight-computer"
  ];
  nativeBuildInputs = [
    makeWrapper
    pkg-config
    python3
  ];

  postFixup = ''
    wrapProgram $out/bin/flight-computer --prefix PATH : ${python3}/bin
  '';
}
