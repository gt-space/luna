{ ... }:
{
  packages = pkgs:
    let
      servo = pkgs.rustPlatform.buildRustPackage {
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
          "--package" "servo"
          "--bin" "servo"
        ];
        nativeBuildInputs = with pkgs; [
          cmake
          pkg-config
          python3
        ];
        buildInputs = with pkgs; [
          openssl
        ];
      };
    in
    {
      servo = { inherit servo; };
    };

  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cmake
        cargoWrapper
        hdf5
        openssl
        pkg-config
        rustToolchain
      ];
    };
  };
}
