{
  flake.modules.homeManager.programs-security-bitwarden = {pkgs, ...}: {
    home.packages = with pkgs; [bitwarden-cli];
  };
}
