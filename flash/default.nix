{ craneLib, pkgs, self, system, ... }:
{
  apps.flash = {
    type = "app";
    program = "${self.packages.${system}.flash}/bin/flash";
  };

  devShells.default = pkgs.mkShell {
    packages = with pkgs; [ rustToolchain ];
  };

  packages.flash = craneLib.buildPackage {
    pname = "flash";
    version = "1.0.0";
    src = craneLib.cleanCargoSource ./.;
  };
}
