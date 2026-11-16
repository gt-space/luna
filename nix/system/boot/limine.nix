{
  flake.modules.nixos.system-boot-limine = {
    boot.loader = {
      efi.canTouchEfiVariables = true;

      limine = {
        enable = true;

        enrollConfig = true;
        panicOnChecksumMismatch = true;

        secureBoot = {
          enable = true;

          autoEnrollKeys.enable = true;
          autoGenerateKeys = true;
        };
      };
    };
  };
}
