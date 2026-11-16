{
  flake.modules.nixos.system-users-groups = {config, ...}: {
    users.users =
      builtins.mapAttrs (name: _: {
        extraGroups = config.home-manager.users.${name}.user.groups;
      })
      config.home-manager.users;
  };

  flake.modules.homeManager.system-users-groups = {lib, ...}: {
    options = {
      user.groups = lib.mkOption {
        type = lib.types.listOf lib.types.str;
        default = [];
      };
    };
  };
}
