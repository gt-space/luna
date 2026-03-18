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

  systemd.network.networks = {
    "10-ethernet".networkConfig.Address = "${devices.ftel.ip}/24";
    "20-radio0".networkConfig.Address = "10.8.8.0/31";
  };
}
