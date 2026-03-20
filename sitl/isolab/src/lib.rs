pub mod args;
pub mod client;
pub mod components;
pub mod lab;
pub mod process;
pub mod scenarios;
pub mod topology;

use anyhow::Result;

pub async fn run() -> Result<()> {
  if std::env::args().nth(1).as_deref() == Some(components::sam::INTERNAL_ARG) {
    return components::sam::run_internal();
  }

  let args = args::Args::parse()?;
  scenarios::run(&args).await?;
  println!("isolab scenario passed: {}", args.scenario.as_str());
  Ok(())
}
