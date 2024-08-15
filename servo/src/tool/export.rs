use serde_json::json;
use std::{fs, path::PathBuf, time::Duration};

/// Function for requesting all data between two timestamps as stored on the
/// ground server.
///
/// Used in the export command line routing.
pub fn export(
  from: Option<f64>,
  to: Option<f64>,
  output_path: &str,
) -> anyhow::Result<()> {
  let output_path = PathBuf::from(output_path);

  let from = from.unwrap_or(0.0);
  let to = to.unwrap_or(f64::MAX);

  let export_format = output_path.extension().unwrap().to_string_lossy();

  let client = reqwest::blocking::Client::new();
  let export_content = client
    .post("http://localhost:7200/data/export")
    .json(&json!({
      "format": export_format,
      "from": from,
      "to": to
    }))
    .timeout(Duration::from_secs(3600))
    .send()?;

  // Either write the file as text if it's a csv, or bytes if it's a file.
  // (assumed for all other returns)
  if export_format == "csv" {
    let text = export_content.text()?;
    fs::write(output_path, text)?;
  } else {
    let bytes = export_content.bytes()?;
    fs::write(output_path, bytes)?;
  }

  Ok(())
}
