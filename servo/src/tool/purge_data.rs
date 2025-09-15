use serde_json::json;

/// Tool function used to send a sequence to be run on the flight computer.
pub fn purge_data() -> anyhow::Result<()> {
  let client = reqwest::blocking::Client::new();
  let response = client
    .post("http://localhost:7200/operator/run-sequence")
    .json(&json!({
      "name": sequence,
      "force": true
    }))
    .send()?;

  println!("{response:#?}");

  Ok(())
}
