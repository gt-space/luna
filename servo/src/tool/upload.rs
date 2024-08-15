use serde_json::json;
use std::{fs, path::Path};

/// Tool function used to upload a sequence to be stored on the control server.
pub fn upload(sequence_path: &Path) -> anyhow::Result<()> {
  let name = sequence_path
    .file_stem()
    .expect("given path does not have a file stem")
    .to_string_lossy()
    .into_owned();

  let script = base64::encode(fs::read(sequence_path)?);

  let client = reqwest::blocking::Client::new();
  let response = client
    .put("http://localhost:7200/operator/sequence")
    .json(&json!({
      "name": name,
      "script": script
    }))
    .send()?;

  println!("{response:#?}");

  Ok(())
}
