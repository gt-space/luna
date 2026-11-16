{inputs, ...}: {
  flake.modules.nixos.hosts-server01 = {
    fileSystems."/" = {
      device = "/dev/mapper/luks-64ddea44-44bc-4784-9820-f2f6b67eae66";
      fsType = "ext4";
    };

    boot.initrd.luks.devices = {
      "luks-64ddea44-44bc-4784-9820-f2f6b67eae66".device = "/dev/disk/by-uuid/64ddea44-44bc-4784-9820-f2f6b67eae66";
      "luks-d57c5444-5e40-4ff0-8a0f-24fadb5301cc".device = "/dev/disk/by-uuid/d57c5444-5e40-4ff0-8a0f-24fadb5301cc";
    };

    fileSystems."/boot" = {
      device = "/dev/disk/by-uuid/5D32-EFE4";
      fsType = "vfat";
      options = ["fmask=0077" "dmask=0077"];
    };

    swapDevices = [
      {device = "/dev/mapper/luks-d57c5444-5e40-4ff0-8a0f-24fadb5301cc";}
    ];

    imports = [inputs.nixos-hardware.nixosModules.lenovo-thinkpad-t490s];

    hardware.facter.reportPath = ./facter.json;
  };
}
