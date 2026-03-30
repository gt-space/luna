{ self, ... }:
let
  devices = builtins.fromJSON (builtins.readFile ../../network.json);
in
{
  imports = [
    self.nixosModules.tel.brain
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
    nftables.ruleset = ''
      table inet mangle {
        chain prerouting {
          type filter hook prerouting priority mangle; policy accept;
          ip daddr ${devices.server-01.ip} ip dscp 46 meta mark set 246
        }
      }

      table ip nat {
        chain postrouting {
          type nat hook postrouting priority 100; policy accept;
          oifname "radio0" snat to 10.8.8.0
        }
      }
    '';
  };

  systemd.network.networks = {
    "10-ethernet".networkConfig.Address = "${devices.ftel.ip}/24";
    "20-radio0" = {
      networkConfig.Address = "10.8.8.0/31";

      routes = [
        {
          routeConfig = {
            Destination = "${devices.server-01.ip}/32";
            Gateway = "10.8.8.1";
            Table = 140;
          };
        }
      ];

      routingPolicyRules = [
        {
          routingPolicyRuleConfig = {
            FirewallMark = 246;
            To = "${devices.server-01.ip}/32";
            Table = 140;
            Priority = 246;
          };
        }
      ];
    };
  };
}
