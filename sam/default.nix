{ ... }:
{
  devShells.default = pkgs: pkgs.mkShell {
    packages = with pkgs; [ rustToolchain ];
  };
}
