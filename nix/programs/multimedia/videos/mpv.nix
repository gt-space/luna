{
  flake.modules.homeManager.programs-multimedia-videos-mpv = {pkgs, ...}: {
    programs.mpv = {
      enable = true;

      config = {
        alang = "en,eng";
        slang = "en,eng";
        vlang = "en,eng";

        force-window = true;
        keep-open = true;

        profile = "high-quality";
        ytdl-format = "bestvideo+bestaudio";

        script-opts = "ytdl_hook-ytdl_path=yt-dlp";

        sub-auto = "fuzzy";
        ytdl-raw-options = ''sub-lang="en,tr",write-sub=,write-auto-sub='';
      };

      scripts =
        (
          with pkgs.mpvScripts; [
            mpris
            sponsorblock
            thumbfast
            uosc
          ]
        )
        ++ (
          with pkgs.mpvScripts.mpv-image-viewer; [
            status-line
            image-positioning
            ruler
            minimap
          ]
        );
    };
  };
}
