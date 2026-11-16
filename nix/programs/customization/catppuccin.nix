let
  catppuccin-config = {
    enable = true;
    autoEnable = true;

    accent = "sapphire";
    flavor = "mocha";
  };
in
  {inputs, ...}: {
    flake.modules.nixos.programs-customization-catppuccin = {
      imports = [inputs.catppuccin.nixosModules.catppuccin];

      catppuccin = catppuccin-config;
    };

    flake.modules.homeManager.programs-customization-catppuccin = {
      imports = [inputs.catppuccin.homeModules.catppuccin];

      catppuccin = catppuccin-config;
    };
  }
