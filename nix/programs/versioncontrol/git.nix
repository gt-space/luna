{
  flake.modules.homeManager.programs-versioncontrol-git = {
    programs.git = {
      enable = true;
      settings = {
        init.defaultBranch = "main";
      };
    };
  };
}
