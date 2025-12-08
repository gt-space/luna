{ ... }:
{
  imports = [
    ../hardware/rpi-4b.nix
  ];

  environment.etc = {
    "ssh/ssh_host_ed25519_key" = {
      mode = "0600";
      source = ../keys/devkit/ed25519.pem;
    };

    "ssh/ssh_host_ed25519_key.pub" = {
      mode = "0600";
      source = ../keys/devkit/ed25519.pub;
    };

    "ssh/ssh_host_rsa_key" = {
      mode = "0600";
      source = ../keys/devkit/rsa.pem;
    };

    "ssh/ssh_host_rsa_key.pub" = {
      mode = "0600";
      source = ../keys/devkit/rsa.pub;
    };
  };

  hardware.deviceTree.overlays = [
    {
      name = "sx1280.dtbo";
      dtsText = ''
        /dts-v1/;
        /plugin/;

        / {
          compatible = "brcm,bcm2711";

          fragment@0 {
            target = <&spi0>;

            __overlay__ {
              status = "okay";

              radio@0 {
                compatible = "semtech,sx1280";
                reg = <0x00>;

                reset-gpios = <&gpio 17 0x01>;
                busy-gpios = <&gpio 22 0x00>;
                dio-gpios = <&gpio 27 0x00>, <0 0>, <0 0>;
              };
            };
          };

          fragment@1 {
            target = <&spidev0>;

            __overlay__ {
              status = "disabled";
            };
          };
        };
      '';
    }
  ];

  networking = {
    hostName = "tel";
    interfaces.eth0.ipv4.addresses = [
      {
        address = "169.254.0.77";
        prefixLength = 16;
      }
    ];
  };
}
