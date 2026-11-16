{
  flake.modules.homeManager.programs-utilities-fzf = {
    programs.fzf = {
      enable = true;
      historyWidget.bash.command = "";
      historyWidget.zsh.command = "";
    };
  };
}
