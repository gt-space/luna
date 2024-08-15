use jeflog::{pass, task};
use std::{env, fs, path::Path};

use crate::tool::deploy;

/// Simple tool function used to clean the servo directory and database.
pub fn clean(servo_dir: &Path) -> anyhow::Result<()> {
  let deployment_cache = deploy::locate_cache()?;
  let mut cache_display = deployment_cache.to_string_lossy().into_owned();

  if cfg!(target_os = "macos") {
    let home = env::var("HOME")?;

    cache_display = cache_display.replace(&home, "~");
  } else if cfg!(target_os = "windows") {
    let app_data = env::var("LOCALAPPDATA")?;

    cache_display = cache_display.replace(&app_data, "%LOCALAPPDATA%");
  }

  task!("Cleaning ~/.servo and {cache_display}.");
  fs::remove_dir_all(servo_dir)?;
  fs::remove_dir_all(deployment_cache)?;
  pass!("Cleaned ~/.servo and {cache_display}.");

  Ok(())
}
