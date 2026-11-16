{
  flake.modules.homeManager.programs-multiplexers-tmux = {pkgs, ...}: {
    catppuccin.tmux.extraConfig = ''
      set -g @catppuccin_window_current_number "#I"
      set -g @catppuccin_window_current_text " #W"
      set -g @catppuccin_window_number "#I"
      set -g @catppuccin_window_text " #W"

      set -g status-right-length 100
      set -g status-left-length 100
      set -g status-left ""
      set -g status-right "#{E:@catppuccin_status_application}"
      set -ag status-right "#{E:@catppuccin_status_session}"
      set -ag status-right "#{E:@catppuccin_status_uptime}"
    '';

    programs.tmux = {
      enable = true;

      baseIndex = 1;
      mouse = true;

      focusEvents = true;
      terminal = "tmux-256color";

      plugins = with pkgs.tmuxPlugins; [
        battery
        cpu

        resurrect
      ];
    };
  };
}
