use serde_json::json;

/// Tool function used to send a sequence to be run on the flight computer.
pub fn purge_states() -> anyhow::Result<()> {
  let client = reqwest::blocking::Client::new();
  let response = client
    .post("http://localhost:7200/data/purge-states")
    .send()?;
  println!("{response:#?}");
  println!("Response body: {}", response.text()?);

  Ok(())
}
