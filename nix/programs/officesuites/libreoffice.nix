{
  flake.modules.homeManager.programs-officesuites-libreoffice = {pkgs, ...}: {
    home.packages = with pkgs; [
      hunspell
      hunspellDicts.en_US
      hyphenDicts.en_US

      libreoffice
    ];
  };
}
