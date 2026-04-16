use crate::{
  args::{Args, Scenario},
  client,
  components::sam,
  process::{self, ManagedChild, ProcessSpec},
  topology::servo_flight::{ServoFlightLab, NS_FLIGHT, NS_SERVO},
};
use anyhow::{bail, Result};
use reqwest::Client;
use std::{
  fs,
  path::Path,
  time::{Duration, Instant},
};
use tokio::time::sleep;

pub async fn run(args: &Args) -> Result<()> {
  match args.scenario {
    Scenario::DefaultSourceUmbilical => run_default_source(args).await,
    Scenario::RadioSurvivesDisconnect => run_radio_survives_disconnect(args).await,
    Scenario::VespulaRadioForwarding => run_vespula_radio(args).await,
    Scenario::RadioWithoutSam => run_radio_without_sam(args).await,
  }
}

struct Harness {
  _servo: ManagedChild,
  _flight: ManagedChild,
  lab: ServoFlightLab,
  sam: Option<ManagedChild>,
}

impl Harness {
  fn new(args: &Args) -> Result<Self> {
    let lab = ServoFlightLab::new(args.workdir.clone())?;
    lab.setup()?;

    let python_dir = process::stage_python_module(&args.workdir, &args.common_lib, "common.so")?;
    let servo_home = args.workdir.join("servo-home");
    let flight_home = args.workdir.join("flight-home");
    fs::create_dir_all(&servo_home)?;
    fs::create_dir_all(&flight_home)?;

    let servo = process::spawn(ProcessSpec {
      namespace: NS_SERVO,
      command: &args.servo_bin,
      args: &["serve", "--volatile", "--quiet"],
      envs: &[("HOME", servo_home.to_str().unwrap())],
      log_path: &args.workdir.join("servo.log"),
    })?;
    let flight = process::spawn(ProcessSpec {
      namespace: NS_FLIGHT,
      command: &args.flight_bin,
      args: &["disable-gps", "desktop"],
      envs: &[
        ("HOME", flight_home.to_str().unwrap()),
        ("PYTHONPATH", python_dir.to_str().unwrap()),
      ],
      log_path: &args.workdir.join("flight.log"),
    })?;

    Ok(Self {
      _servo: servo,
      _flight: flight,
      lab,
      sam: None,
    })
  }

  fn start_sam(&mut self) -> Result<()> {
    let current_exe = std::env::current_exe()?;
    self.sam = Some(process::spawn(ProcessSpec {
      namespace: NS_FLIGHT,
      command: &current_exe,
      args: &[sam::INTERNAL_ARG],
      envs: &[],
      log_path: Path::new("/tmp/isolab-sam.log"),
    })?);
    Ok(())
  }
}

async fn wait_for_flight_connection() -> Result<()> {
  let deadline = Instant::now() + Duration::from_secs(30);
  while Instant::now() < deadline {
    let output = std::process::Command::new("ip")
      .args([
        "netns", "exec", NS_SERVO, "ss", "-tnH", "state", "established", "sport", "=", ":5025",
      ])
      .output()?;
    if !String::from_utf8_lossy(&output.stdout).trim().is_empty() {
      return Ok(());
    }
    sleep(Duration::from_millis(200)).await;
  }
  bail!("flight did not establish TCP connection to servo");
}

async fn setup(args: &Args) -> Result<(Harness, Client)> {
  let mut harness = Harness::new(args)?;
  let client = Client::builder().timeout(Duration::from_secs(5)).build()?;
  client::wait_for_http(&client).await?;
  wait_for_flight_connection().await?;
  let mappings = client::build_mappings();
  client::configure_servo(&client, &mappings).await?;
  harness.start_sam()?;
  Ok((harness, client))
}

async fn setup_without_sam(args: &Args) -> Result<(Harness, Client)> {
  let harness = Harness::new(args)?;
  let client = Client::builder().timeout(Duration::from_secs(5)).build()?;
  client::wait_for_http(&client).await?;
  wait_for_flight_connection().await?;
  let mappings = client::build_mappings();
  client::configure_servo(&client, &mappings).await?;
  Ok((harness, client))
}

fn assert_ground_only_mappings_visibility(
  umbilical: &common::comm::VehicleState,
  radio: &common::comm::VehicleState,
) -> Result<()> {
  for channel in 1..=10u32 {
    let valve_name = format!("GSAM_VLV{channel:02}");
    let sensor_name = format!("GSAM_PT{channel:02}");

    anyhow::ensure!(
      umbilical.valve_states.contains_key(&valve_name),
      "expected umbilical telemetry to include ground valve mapping {valve_name}",
    );
    anyhow::ensure!(
      umbilical.sensor_readings.contains_key(&sensor_name),
      "expected umbilical telemetry to include ground sensor mapping {sensor_name}",
    );
    anyhow::ensure!(
      !radio.valve_states.contains_key(&valve_name),
      "expected radio telemetry to omit ground valve mapping {valve_name}",
    );
    anyhow::ensure!(
      !radio.sensor_readings.contains_key(&sensor_name),
      "expected radio telemetry to omit ground sensor mapping {sensor_name}",
    );
  }

  Ok(())
}

async fn run_default_source(args: &Args) -> Result<()> {
  let (harness, _) = setup(args).await?;

  let mut default_ws = client::connect(None).await?;
  let mut umbilical_ws = client::connect(Some("umbilical")).await?;
  let mut radio_ws = client::connect(Some("tel")).await?;

  let default_state = client::wait_for_expected_state(&mut default_ws, 20, 22).await?;
  let umbilical_state = client::wait_for_expected_state(&mut umbilical_ws, 20, 22).await?;
  let radio_state = client::wait_for_expected_state(&mut radio_ws, 10, 12).await?;

  client::assert_expected_shape(&default_state, 20, 22, 20)?;
  client::assert_expected_shape(&umbilical_state, 20, 22, 20)?;
  client::assert_expected_shape(&radio_state, 10, 12, 0)?;
  assert_ground_only_mappings_visibility(&default_state, &radio_state)?;
  assert_ground_only_mappings_visibility(&umbilical_state, &radio_state)?;

  harness.lab.toggle_umbilical(false)?;

  let default_stale = client::wait_for_repeated_state(&mut default_ws, Duration::from_secs(4)).await?;
  let umbilical_stale = client::wait_for_repeated_state(&mut umbilical_ws, Duration::from_secs(4)).await?;
  let radio_after_disconnect =
    client::wait_for_changed_state(&mut radio_ws, &radio_state, Duration::from_secs(3)).await?;

  client::assert_expected_shape(&default_stale, 20, 22, 20)?;
  client::assert_expected_shape(&umbilical_stale, 20, 22, 20)?;
  client::assert_expected_shape(&radio_after_disconnect, 10, 12, 0)?;

  Ok(())
}

async fn run_radio_survives_disconnect(args: &Args) -> Result<()> {
  let (harness, _) = setup(args).await?;

  let mut umbilical_ws = client::connect(Some("umbilical")).await?;
  let mut radio_ws = client::connect(Some("tel")).await?;

  let umbilical_state = client::wait_for_expected_state(&mut umbilical_ws, 20, 22).await?;
  let radio_state = client::wait_for_expected_state(&mut radio_ws, 10, 12).await?;

  client::assert_expected_shape(&umbilical_state, 20, 22, 20)?;
  client::assert_expected_shape(&radio_state, 10, 12, 0)?;
  assert_ground_only_mappings_visibility(&umbilical_state, &radio_state)?;

  harness.lab.toggle_umbilical(false)?;

  let radio_after_disconnect =
    client::wait_for_changed_state(&mut radio_ws, &radio_state, Duration::from_secs(3)).await?;
  let stale_umbilical =
    client::wait_for_repeated_state(&mut umbilical_ws, Duration::from_secs(4)).await?;

  client::assert_expected_shape(&radio_after_disconnect, 10, 12, 0)?;
  client::assert_expected_shape(&stale_umbilical, 20, 22, 20)?;

  Ok(())
}

async fn run_vespula_radio(args: &Args) -> Result<()> {
  let (_harness, _) = setup(args).await?;
  let mappings = client::build_mappings();
  anyhow::ensure!(
    client::count_valve_helper_sensors(&mappings) == 20,
    "expected the Vespula SITL mapping set to include 20 valve helper sensors, found {}",
    client::count_valve_helper_sensors(&mappings),
  );
  anyhow::ensure!(
    client::count_non_radio_mappings(&mappings) == 20,
    "expected the Vespula SITL mapping set to include 20 non-radio mappings, found {}",
    client::count_non_radio_mappings(&mappings),
  );

  let mut umbilical_ws = client::connect(Some("umbilical")).await?;
  let mut radio_ws = client::connect(Some("tel")).await?;
  let umbilical_state = client::wait_for_expected_state(&mut umbilical_ws, 20, 22).await?;
  let radio_state = client::wait_for_expected_state(&mut radio_ws, 10, 12).await?;
  let advanced_state =
    client::wait_for_changed_state(&mut radio_ws, &radio_state, Duration::from_secs(3)).await?;

  client::assert_expected_shape(&umbilical_state, 20, 22, 20)?;
  client::assert_expected_shape(&radio_state, 10, 12, 0)?;
  client::assert_expected_shape(&advanced_state, 10, 12, 0)?;
  assert_ground_only_mappings_visibility(&umbilical_state, &radio_state)?;

  Ok(())
}

async fn run_radio_without_sam(args: &Args) -> Result<()> {
  let (_harness, _) = setup_without_sam(args).await?;
  let mappings = client::build_mappings();
  anyhow::ensure!(
    client::count_valve_helper_sensors(&mappings) == 20,
    "expected the no-SAM SITL mapping set to include 20 valve helper sensors, found {}",
    client::count_valve_helper_sensors(&mappings),
  );

  let mut radio_ws = client::connect(Some("tel")).await?;
  let radio_state = client::wait_for_expected_state(&mut radio_ws, 10, 12).await?;
  client::assert_expected_shape(&radio_state, 10, 12, 0)?;

  Ok(())
}
