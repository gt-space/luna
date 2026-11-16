{
  flake.modules.homeManager.programs-security-ssh = {config, ...}: {
    programs.ssh = {
      enable = true;
      enableDefaultConfig = false;
      settings."*" = {
        ForwardAgent = false;
        Compression = false;
        HashKnownHosts = true;

        ServerAliveInterval = 30;
        ServerAliveCountMax = 3;

        AddKeysToAgent = "no";

        UserKnownHostsFile = "~/.ssh/known_hosts";

        ControlMaster = "no";
        ControlPath = "~/.ssh/master-%r@%n:%p";
        ControlPersist = "no";
      };
    };
  };
}
