{
  flake.modules.nixos.programs-networking-wireshark = {
    programs.wireshark.enable = true;
  };

  flake.modules.homeManager.programs-networking-wireshark = {
    user.groups = ["wireshark"];
  };
}
