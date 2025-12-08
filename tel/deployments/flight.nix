{ ... }:
{
  imports = [
    ../hardware/rpi-cm4.nix
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

  hardware.deviceTree.overlays = [
    {
      name = "sx1280.dtbo";
      dtsText = ''
        /dts-v1/;
        /plugin/;

        / {
          compatible = "brcm,bcm2711";

          fragment@0 {
            target = <&gpio>;
            __overlay__ {
              spi1_pins: spi1_pins {
                brcm,pins = <19 20 21>;
                brcm,function = <3>; /* BCM2835_FSEL_ALT4 */
              };

              spi1_cs_pins: spi1_cs_pins {
                brcm,pins = <18 17>;
                brcm,function = <1>; /* BCM2835_FSEL_GPIO_OUT */
              };
            };
          };

          fragment@1 {
            target = <&spi1>;
            __overlay__ {
              pinctrl-names = "default";
              pinctrl-0 = <&spi1_pins &spi1_cs_pins>;
              cs-gpios = <&gpio 18 1>, <&gpio 17 1>;
              status = "okay";

              radio@0 {
                compatible = "semtech,sx1280";
                reg = <1>;
                spi-max-frequency = <5000000>;

                busy-gpios = <&gpio 27 0x00>;
                dio2-gpios = <&gpio 5 0x00>;
                dio3-gpios = <&gpio 22 0x00>;
                reset-gpios = <&gpio 6 0x01>;
              };
            };
          };
        };
      '';
    }
  ];

  networking = {
    hostName = "ftel";

    interfaces = {
      eth0.ipv4.addresses = [
        {
          address = "192.168.1.132";
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
  };
}
