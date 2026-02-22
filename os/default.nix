{ pkgs, ... }:
let
  armPkgs = pkgs.pkgsCross.armv7l-hf-multiplatform;
in
{
  packages.am335x-usb-bootloader = armPkgs.buildUBoot {
    defconfig = "am335x_evm_defconfig";
    extraMeta.platforms = [ "armv7l-linux" ];
    filesToInstall = [ "spl/u-boot-spl.bin" "u-boot.img" ];

    extraConfig = ''
      CONFIG_BOOTCOMMAND="unbind ethernet 3; ums 0 mmc 1"
      CONFIG_SPL_MTD_SUPPORT=n
      CONFIG_SPL_NAND_SUPPORT=n
      CONFIG_SPL_SPI_SUPPORT=n
      CONFIG_SPL_I2C=y
      CONFIG_SPL_GPIO=y
      CONFIG_SPL_POWER=y
      CONFIG_SPL_NET=y
      CONFIG_SPL_ETH=y
      CONFIG_SPL_USB_GADGET=y
      CONFIG_SPL_USB_ETHER=y
      CONFIG_SPL_DM_USB_GADGET=y
      CONFIG_SPL_MUSB_NEW=y
      CONFIG_DM_USB_GADGET=y
      CONFIG_USB_GADGET=y
      CONFIG_USB_ETHER=y
      CONFIG_USB_MUSB_GADGET=y
      CONFIG_USB_MUSB_TI=y
      CONFIG_CMD_USB_MASS_STORAGE=y
      CONFIG_USB_FUNCTION_MASS_STORAGE=y
    '';
  };
}
