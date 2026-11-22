{
  flake.modules.nixos.hosts-server01 = {
    # network profile to automatically set static ip of ethernet peripherals
    # ip for server01 as per "Network Allocation" is 192.168.1.10
    networking.networkManager.ensureProfiles = {
      profiles.static-ip = {
        connection = {
          id = "Ethernet Static IP";
          type = "ethernet";
          autoconnect = true;
        };
        match.interface-name = "enx*";
        ipv4 = {
          method = "manual";
          address1 = "192.168.1.10/24";
        };
        ipv6.method = "disabled";
      };
    };
  };
}
