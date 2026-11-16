{
  flake.modules.homeManager.programs-utilities-atuin = {config, ...}: {
    programs.atuin = {
      enable = true;
      daemon.enable = true;

      flags = ["--disable-up-arrow"];
    };
  };
}
