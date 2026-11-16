{inputs, ...}: {
  flake.modules.nixos.system-users-homemanager = {config, ...}: {
    imports = [inputs.home-manager.nixosModules.home-manager];

    home-manager = {
      useGlobalPkgs = true;
      useUserPackages = true;
    };
  };
}
