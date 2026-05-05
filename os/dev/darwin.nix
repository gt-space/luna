{ nix-darwin, self, ... }:
{
  darwinConfigurations.dev = nix-darwin.lib.darwinSystem {
    system = "aarch64-darwin";
    modules = [
      self.darwinModules.dev-builders
    ];
  };

  darwinModules.dev-builders = { ... }: {
    nix = {
      enable = true;

      settings = {
        builders-use-substitutes = true;
        experimental-features = [ "nix-command" "flakes" ];
        trusted-users = [ "@admin" ];
      };

      linux-builder = {
        enable = true;

        systems = [
          "aarch64-linux"
          "x86_64-linux"
        ];

        config = { ... }: {
          boot.binfmt.emulatedSystems = [ "x86_64-linux" ];

          virtualisation = {
            cores = 4;

            darwin-builder = {
              memorySize = 8192;
              diskSize = 40 * 1024;
              hostPort = 31022;
            };
          };
        };
      };
    };
  };
}
