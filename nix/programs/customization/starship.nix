{
  flake.modules.homeManager.programs-customization-starship = {
    programs.starship = {
      enable = true;

      presets = ["nerd-font-symbols"];

      settings = {
        format = "$all$shlvl$character";

        cmd_duration = {
          min_time = 0;
          show_milliseconds = true;
        };

        directory = {
          truncation_length = 99;
        };

        git_status = {
          ahead = "";
          behind = "";
          deleted = "X";
          diverged = "󰹹";
        };

        hostname = {
          ssh_only = false;
          ssh_symbol = " through [ssh](bold dimmed blue)";
          format = "[$hostname]($style)$ssh_symbol in ";
        };

        shlvl = {
          disabled = false;
          symbol = "❯";
          repeat = true;
          repeat_offset = 1;
          format = "[$symbol]($style)";
        };

        status = {
          disabled = false;
          format = "\\[[($common_meaning$signal_name )$int]($style)\\] ";
          pipestatus = true;
          pipestatus_separator = " | ";
          pipestatus_format = "\\[$pipestatus\\] 󰜴 \\[[($common_meaning$signal_name )$int]($style)\\] ";
          pipestatus_segment_format = "[($common_meaning$signal_name )$int]($style)";
        };

        username = {
          show_always = true;
          format = "[$user]($style)@";
        };
      };
    };
  };
}
