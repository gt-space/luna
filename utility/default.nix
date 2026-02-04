{ pkgs, ... }:
{
  devShells.default = pkgs.mkShell {
    packages = with pkgs; [ rust ];
  };
}
