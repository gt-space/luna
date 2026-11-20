{
  perSystem = {
    config,
    pkgs,
    ...
  }: {
    devShells.default = pkgs.mkShell {
      strictDeps = true;

      # merge all devshells
      inputsFrom = with config.devShells; [
        bms
        common
        flight2
        gui
        sam
        servo
        tel
        utility
      ];

      # lsps for config files
      nativeBuildInputs = with pkgs; [
        alejandra
        nil

        bash-language-server

        lua-language-server

        taplo

        vscode-json-languageserver

        yaml-language-server
      ];
    };
  };
}
