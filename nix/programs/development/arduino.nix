{
  flake.modules.homeManager.programs-development-arduino = {pkgs, ...}: {
    home.packages = with pkgs; [arduino-ide];
    user.groups = ["dialout"];
  };
}
