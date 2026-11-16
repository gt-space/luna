{
  self,
  inputs,
  ...
}: {
  flake.modules.nixos.programs-security-sops = {config, ...}: {
    imports = [
      inputs.sops-nix.nixosModules.sops
    ];

    sops = {
      age.sshKeyPaths = ["/etc/ssh/ssh_host_ed25519_key"];
      defaultSopsFile = "${self}/secrets/root/${config.networking.hostName}/secrets.yaml";
    };
  };

  flake.modules.homeManager.programs-security-sops = {
    config,
    osConfig,
    pkgs,
    ...
  }: {
    imports = [
      inputs.sops-nix.homeManagerModules.sops
    ];

    home.packages = with pkgs; [sops];

    sops = {
      age.keyFile = "${config.xdg.configHome}/sops/age/keys.txt";
      defaultSopsFile = "${self}/secrets/home/${osConfig.networking.hostName}/secrets.yaml";
    };
  };
}
