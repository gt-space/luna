{
  flake.modules.homeManager.programs-organization-zoxide = {
    programs.zoxide = {
      enable = true;
      options = ["--cmd cd"];
    };
  };
}
