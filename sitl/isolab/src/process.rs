use anyhow::{bail, Result};
use std::{
  fs,
  path::{Path, PathBuf},
  process::{Child, Command, Stdio},
};

pub struct ManagedChild(pub Child);

impl Drop for ManagedChild {
  fn drop(&mut self) {
    let _ = self.0.kill();
    let _ = self.0.wait();
  }
}

pub struct ProcessSpec<'a> {
  pub namespace: &'a str,
  pub command: &'a Path,
  pub args: &'a [&'a str],
  pub envs: &'a [(&'a str, &'a str)],
  pub log_path: &'a Path,
}

pub fn spawn(spec: ProcessSpec<'_>) -> Result<ManagedChild> {
  let log = fs::File::create(spec.log_path)?;
  let log_err = log.try_clone()?;

  let mut cmd = Command::new("ip");
  cmd.args(["netns", "exec", spec.namespace, "env"]);
  for (key, value) in spec.envs {
    cmd.arg(format!("{key}={value}"));
  }
  cmd.arg(spec.command);
  cmd.args(spec.args);
  cmd.stdout(Stdio::from(log));
  cmd.stderr(Stdio::from(log_err));
  Ok(ManagedChild(cmd.spawn()?))
}

pub fn stage_python_module(workdir: &Path, source: &Path, module_name: &str) -> Result<PathBuf> {
  let python_dir = workdir.join("python");
  fs::create_dir_all(&python_dir)?;
  fs::copy(source, python_dir.join(module_name))?;
  Ok(python_dir)
}

pub fn run_command(args: &[&str]) -> Result<()> {
  let status = Command::new(args[0]).args(&args[1..]).status()?;
  if !status.success() {
    bail!("command failed: {}", args.join(" "));
  }
  Ok(())
}
