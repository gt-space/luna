{ ... }:
let
  devices = builtins.fromJSON (builtins.readFile ../../network.json);
in
{
  environment.etc = {
    "ssh/ssh_host_ed25519_key" = {
      mode = "0600";
      source = ../keys/ground/ed25519.pem;
    };

    "ssh/ssh_host_ed25519_key.pub" = {
      mode = "0600";
      source = ../keys/ground/ed25519.pub;
    };

    "ssh/ssh_host_rsa_key" = {
      mode = "0600";
      source = ../keys/ground/rsa.pem;
    };

    "ssh/ssh_host_rsa_key.pub" = {
      mode = "0600";
      source = ../keys/ground/rsa.pub;
    };
  };

  networking = {
    hostName = "gtel";
    interfaces = {
      eth0.ipv4.addresses = [
        {
          address = devices.gtel.ip;
          prefixLength = 24;
        }
      ];

      radio0.ipv4.addresses = [
        {
          address = "10.8.8.1";
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
            iifname "radio0" dnat to ${devices.server-01.ip}
          }

          chain postrouting {
            type nat hook postrouting priority 100; policy accept;
            oifname "eth0" ip daddr ${devices.server-01.ip} snat to ${devices.gtel.ip}
          }
        }
      '';
    };
  };
}
