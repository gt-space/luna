{
  flake.modules.homeManager.programs-shells-bash = {
    home.shell.enableBashIntegration = true;

    programs.bash = {
      enable = true;

      shellOptions = [
        "globstar"
        "failglob"
      ];
    };
  };
}
