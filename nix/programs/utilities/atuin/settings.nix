{
  flake.modules.homeManager.programs-utilities-atuin = {
    programs.atuin.settings = {
      ai.enabled = false;
      auto_sync = true;
      enter_accept = false;
      history_filter = [
        "base64"
        "base32"
        "^zi$"
        "echo .* wg pubkey"
        "/nix/store"
        "CLOUDFLARE_API_TOKEN"
        "api_token"
        "API_TOKEN"
        "^ "
      ];
      keymap_mode = "vim-insert";
      prefers_reduced_motion = true;
      search_mode = "fuzzy";
      show_help = false;
      stats = {
        common_subcommands = [
          "git"
          "nix"
          "tmux"
          "systemctl"
        ];
        ignored_commands = [
          "cd"
          "fg"
          "ls"
          "nvim"
          "z"
        ];
      };
      style = "compact";
      sync.records = true;
      sync_frequency = "1h";
      sync_address = "https://api.atuin.sh";
      tmux.enabled = true;
      update_check = false;
    };
  };
}
