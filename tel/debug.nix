{ config, lib, pkgs, sx1280, ... }:
let
  kernel = config.boot.kernelPackages.kernel;
  kdir = "${kernel.dev}/lib/modules/${kernel.modDirVersion}/build";
in
{
  imports = [
    ./release.nix
  ];

  environment = {
    systemPackages = with pkgs; [
      # Hardware debugging tools
      libraspberrypi
      raspberrypi-eeprom
      usbutils

      # Dev tools
      autoconf
      bison
      ccache
      cmake
      curl
      dtc
      file
      flex
      gcc
      git
      gnumake
      gnupg
      kmod
      libgpiod

      # Kernel header files
      config.boot.kernelPackages.kernel.dev

      # Python packages
      (python312.withPackages (ps: with ps; [
        spidev
        rpi-gpio
      ]))
    ];

    variables.KERNELDIR = kdir;
  };

  # Override the release settings to re-enable Nix on-device.
  nix = {
    enable = lib.mkForce true;
    settings.experimental-features = [ "nix-command" "flakes" ];
  };

  system.activationScripts.copySX1280 = {
    text = ''
      rm -rf /home/yjsp/sx1280
      cp -r ${sx1280} /home/yjsp/sx1280
      chmod -R u+w /home/yjsp/sx1280
      chown -R yjsp:users /home/yjsp/sx1280 2>/dev/null || true
    '';

    deps = [ "users" ];
  };
}
