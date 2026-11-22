{
  flake.modules.nixos.programs-networking-arpscan = {
    programs.arp-scan.enable = true;
  };
}
