use anyhow::{bail, Context, Result};
use std::path::PathBuf;

#[derive(Debug)]
pub struct Args {
  pub servo_bin: PathBuf,
  pub flight_bin: PathBuf,
  pub common_lib: PathBuf,
  pub workdir: PathBuf,
  pub scenario: Scenario,
}

#[derive(Clone, Copy, Debug)]
pub enum Scenario {
  DefaultSourceUmbilical,
  RadioSurvivesDisconnect,
  VespulaRadioForwarding,
  RadioWithoutSam,
}

impl Args {
  pub fn parse() -> Result<Self> {
    let mut servo_bin = None;
    let mut flight_bin = None;
    let mut common_lib = None;
    let mut workdir = None;
    let mut scenario = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
      match arg.as_str() {
        "--servo-bin" => servo_bin = args.next().map(PathBuf::from),
        "--flight-bin" => flight_bin = args.next().map(PathBuf::from),
        "--common-lib" => common_lib = args.next().map(PathBuf::from),
        "--workdir" => workdir = args.next().map(PathBuf::from),
        "--scenario" => {
          let value = args.next().context("--scenario requires a value")?;
          scenario = Some(Scenario::parse(&value)?);
        }
        other => bail!("unrecognized argument: {other}"),
      }
    }

    Ok(Self {
      servo_bin: servo_bin.context("--servo-bin is required")?,
      flight_bin: flight_bin.context("--flight-bin is required")?,
      common_lib: common_lib.context("--common-lib is required")?,
      workdir: workdir.unwrap_or_else(|| PathBuf::from("/tmp/isolab")),
      scenario: scenario.context("--scenario is required")?,
    })
  }
}

impl Scenario {
  pub fn parse(value: &str) -> Result<Self> {
    match value {
      "default-source-umbilical" => Ok(Self::DefaultSourceUmbilical),
      "radio-survives-disconnect" => Ok(Self::RadioSurvivesDisconnect),
      "vespula-radio-forwarding" => Ok(Self::VespulaRadioForwarding),
      "radio-without-sam" => Ok(Self::RadioWithoutSam),
      _ => bail!("unknown scenario: {value}"),
    }
  }

  pub fn as_str(self) -> &'static str {
    match self {
      Self::DefaultSourceUmbilical => "default-source-umbilical",
      Self::RadioSurvivesDisconnect => "radio-survives-disconnect",
      Self::VespulaRadioForwarding => "vespula-radio-forwarding",
      Self::RadioWithoutSam => "radio-without-sam",
    }
  }
}
