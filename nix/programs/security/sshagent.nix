{
  flake.modules.homeManager.programs-security-sshagent = {
    services.ssh-agent.enable = true;
  };
}
