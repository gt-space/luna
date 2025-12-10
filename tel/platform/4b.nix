{ nixos-hardware, pkgs, ... }:
{
  imports = [
    nixos-hardware.nixosModules.raspberry-pi-4
  ];

  boot = {
    kernelPackages = pkgs.linuxPackages_rpi4;
    initrd.allowMissingModules = true;
  };

  hardware = {
    deviceTree = {
      dtbSource = pkgs.device-tree_rpi;
      enable = true;
      filter = "bcm2711-rpi-4-b.dtb";
      name = "broadcom/bcm2711-rpi-4-b.dtb";

      # Apply device tree overlays according to the development kit pinout
      # provided in the README.
      overlays = [
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
                    reg = <0>;
                    spi-max-frequency = <5000000>;

                    reset-gpios = <&gpio 17 0x01>;
                    busy-gpios = <&gpio 22 0x00>;
                    dio1-gpios = <&gpio 27 0x00>;
                  };
                };
              };

              fragment@1 {
                target = <&spidev0>;
                __overlay__ {
                  status = "disabled";
                };
              };

              fragment@2 {
                target = <&spidev1>;
                __overlay__ {
                  status = "disabled";
                };
              };
            };
          '';
        }
      ];
    };

    raspberry-pi."4".apply-overlays-dtmerge.enable = true;
  };
}
