{ pkgs, ... }:
{
  imports = [
    ../../os/platform/beaglebone-black.nix
    ../../os/profiles/embedded.nix
    ../../os/users/yjsp.nix
  ];

  boot = {
    # Partition growth handled by sdImage.expandOnBoot (default: true)
    # which uses shell scripts instead of cloud-utils + Python
    initrd.availableKernelModules = [ "mmc_block" "ext4" ];
  };

  networking = {
    hostName = "sam";
    defaultGateway = "192.168.1.1";
    firewall.enable = false;
    # Disable dhcpcd to prevent duplicate systemd package in closure
    dhcpcd.enable = false;
    useDHCP = false;
  };

  # Use systemd-networkd for static networking instead of dhcpcd
  systemd.network = {
    enable = true;
    networks."10-eth0" = {
      matchConfig.Name = "eth0";
      networkConfig = {
        Address = "192.168.1.100/24";  # Adjust to your desired static IP
        Gateway = "192.168.1.1";
        DNS = "8.8.8.8";
      };
    };
  };

  security = {
    pam.services.sshd.allowNullPassword = true;
    sudo.wheelNeedsPassword = false;
  };

  services.openssh = {
    enable = true;
    settings = {
      PermitEmptyPasswords = "yes";
      PermitRootLogin = "yes";
    };
  };

  system.stateVersion = "25.11";

  systemd.services.sam = {
    description = "SAM Runtime";
    wantedBy = [ "multi-user.target" ];
    after = [ "network-online.target" ];
    wants = [ "network-online.target" ];
    serviceConfig = {
      ExecStart = "${pkgs.sam-runtime}/bin/sam";
      Restart = "on-failure";
    };
  };
}
