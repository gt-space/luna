{ ... }:
{
  packages = pkgs:
    let
      flight-computer = pkgs.rustPlatform.buildRustPackage {
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
          "--package" "flight-computer"
          "--bin" "flight-computer"
        ];
        nativeBuildInputs = with pkgs; [
          pkg-config
          python3
        ];
      };
    in
    {
      flight2 = { inherit flight-computer; };
    };

  devShells = pkgs: {
    default = pkgs.mkShell {
      packages = with pkgs; [
        cargoWrapper
        rustToolchain
      ];
    };
  };
}
