{
  flake.modules.nixos.programs-shells-zsh = {
    programs.zsh.enable = true;
  };

  flake.modules.homeManager.programs-shells-zsh = {config, ...}: {
    home.shell.enableZshIntegration = true;

    programs.zsh = {
      enable = true;
      dotDir = "${config.xdg.configHome}/zsh";
      enableCompletion = true;
      autosuggestion = {
        enable = true;
        strategy = [];
      };
      history = {
        size = 10000;
        save = 10000;
      };

      setOptions = [
        "EXTENDED_GLOB"
        "GLOB_STAR_SHORT"
        "NOMATCH"
      ];
    };
  };
}
