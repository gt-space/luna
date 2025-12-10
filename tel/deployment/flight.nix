{ brain, ... }:
let
  devices = builtins.fromJSON (builtins.readFile ../../network.json);
in
{
  imports = [
    brain.nixosModules.default
  ];

  environment.etc = {
    "ssh/ssh_host_ed25519_key" = {
      mode = "0600";
      source = ../keys/flight/ed25519.pem;
    };

    "ssh/ssh_host_ed25519_key.pub" = {
      mode = "0600";
      source = ../keys/flight/ed25519.pub;
    };

    "ssh/ssh_host_rsa_key" = {
      mode = "0600";
      source = ../keys/flight/rsa.pem;
    };

    "ssh/ssh_host_rsa_key.pub" = {
      mode = "0600";
      source = ../keys/flight/rsa.pub;
    };
  };

  networking = {
    hostName = "ftel";

    interfaces = {
      eth0.ipv4.addresses = [
        {
          address = devices.ftel.ip;
          prefixLength = 24;
        }
      ];

      radio0.ipv4.addresses = [
        {
          address = "10.8.8.0";
          prefixLength = 31;
        }
      ];
    };

    nftables = {
      enable = true;
      ruleset = ''
        table ip nat {
          chain prerouting {
            type nat hook prerouting priority -100; policy accept;
            ip saddr ${devices.flight.ip} ip daddr ${devices.server-01.ip} ip dscp 10 dnat to 10.8.8.1
          }

          chain postrouting {
            type nat hook postrouting priority 100; policy accept;
            oifname "radio0" snat to 10.8.8.0
          }
        }
      '';
    };
  };
}
