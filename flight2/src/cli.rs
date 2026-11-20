use std::{collections::HashSet, path::PathBuf};

use clap::{error::ErrorKind, CommandFactory, Parser, ValueEnum};

use crate::{
    config_readout::Section,
    file_logger::LoggerConfig,
    lidar::{DistanceUnit, LidarConfig},
};

/// Runtime commands for the flight computer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum SensorCommands {
    /// Disable all local sensor workers
    Desktop,
    /// Disable the GPS/RECO worker
    DisableGpsReco,
    /// Disable the FC-local IMU
    DisableImu,
    /// Disable the FC-local magnetometer
    DisableMagnetometer,
    /// Disable the FC-local barometer
    DisableBarometer,
}

/// State that describes which local workers should be active or not
/// Workers are threads that are constantly collecting data from sensors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorkerConfig {
    desktop_mode: bool,
    gps_reco_enabled: bool,
    imu_enabled: bool,
    magnetometer_enabled: bool,
    barometer_enabled: bool,
}

impl Default for WorkerConfig {
    fn default() -> Self {
        Self {
            desktop_mode: false,
            gps_reco_enabled: true,
            imu_enabled: true,
            magnetometer_enabled: true,
            barometer_enabled: true,
        }
    }
}

impl WorkerConfig {
    /// Parses `SensorCommands` instance and returns the computed WorkerConfig.
    fn from_cli_commands(commands: &[SensorCommands]) -> Self {
        let mut plan = Self::default();
        for command in commands {
            match command {
                SensorCommands::Desktop => {
                    plan.desktop_mode = true;
                    plan.gps_reco_enabled = false;
                    plan.imu_enabled = false;
                    plan.magnetometer_enabled = false;
                    plan.barometer_enabled = false;
                }
                SensorCommands::DisableGpsReco => plan.gps_reco_enabled = false,
                SensorCommands::DisableImu => plan.imu_enabled = false,
                SensorCommands::DisableMagnetometer => plan.magnetometer_enabled = false,
                SensorCommands::DisableBarometer => plan.barometer_enabled = false,
            }
        }

        plan
    }

    /// Returns `True` if we are in desktop mode, else 'False'
    pub fn desktop_mode(&self) -> bool {
        self.desktop_mode
    }

    /// Returns `True` if the GPS/RECO worker should be enabled, else 'False'
    pub fn gps_reco_enabled(&self) -> bool {
        self.gps_reco_enabled
    }

    /// Returns `True` if we should be collecting IMU data, else 'False'
    pub fn imu_enabled(&self) -> bool {
        self.imu_enabled
    }

    /// Returns `True` if we should be collecting magnetometer data, else
    /// 'False'
    pub fn magnetometer_enabled(&self) -> bool {
        self.magnetometer_enabled
    }

    /// Returns `True` if we should be collecting barometer data, else 'False'
    pub fn barometer_enabled(&self) -> bool {
        self.barometer_enabled
    }

    /// Returns `True` if the mag / bar worker should be enabled, else 'False'
    pub fn mag_bar_enabled(&self) -> bool {
        self.magnetometer_enabled || self.barometer_enabled
    }
}

#[derive(Debug)]
pub struct RuntimeConfig {
    pub worker_config: WorkerConfig,
    pub logger_config: LoggerConfig,
    pub print_gps: bool,
    pub show_config: Option<Vec<Section>>,
    pub lidars: Vec<LidarConfig>,
    pub lidar_unit: DistanceUnit,
    /// `--sim-plant`: fly the precompiled plant on the bench
    #[cfg(feature = "sim-plant")]
    pub sim_plant: Option<crate::sim_plant::SimPlantConfig>,
}

impl RuntimeConfig {
    fn from_args(args: Args) -> Self {
        Self {
            worker_config: WorkerConfig::from_cli_commands(&args.commands),
            logger_config: LoggerConfig::from_cli_commands(
                args.disable_file_logging,
                args.log_dirs,
                args.log_buffer_size,
                args.log_rotation_mb,
                args.fsync_rate,
            ),
            print_gps: args.print_gps,
            show_config: args.show_config,
            lidars: args.lidars,
            lidar_unit: args.lidar_unit,
            #[cfg(feature = "sim-plant")]
            sim_plant: args
                .sim_plant
                .map(|gains| crate::sim_plant::SimPlantConfig {
                    gains,
                    config: args.sim_config,
                    log: args.sim_log,
                }),
        }
    }
}

/// Command-line arguments for the flight computer.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Stackable runtime commands such as `disable-imu disable-magnetometer`
    #[arg(value_enum, value_name = "COMMAND")]
    commands: Vec<SensorCommands>,

    /// Disable file logging (enabled by default)
    #[arg(long, default_value_t = false, global = true)]
    disable_file_logging: bool,

    /// Directories for log files (default: $HOME/flight_logs)
    #[arg(long, global = true)]
    log_dirs: Option<Vec<PathBuf>>,

    /// Buffer size in samples (default: 100)
    #[arg(long, default_value_t = 100, global = true)]
    log_buffer_size: usize,

    /// File rotation size threshold in MB (default: 100)
    #[arg(long, default_value_t = 100, global = true)]
    log_rotation_mb: u64,

    /// Print GPS data to terminal at ~1Hz (disabled by default)
    #[arg(long, default_value_t = false, global = true)]
    print_gps: bool,

    /// How often log data should be written to disk after flushing the internal
    /// buffer `fsync_rate` times (disabled by default)
    #[arg(long, default_value_t = 0, global = false)]
    fsync_rate: usize,

    /// Print config readout and exit, prints all sections if none are specified
    #[arg(long, value_enum, num_args = 0.., require_equals = true, value_delimiter = ',')]
    show_config: Option<Vec<Section>>,

    /// TF-Luna bridge channel. Repeat for 2–4 sensors (default: disabled).
    #[arg(long = "lidar", value_name = "ID=udp://IP:PORT or ID=tcp://IP:PORT")]
    lidars: Vec<LidarConfig>,

    /// Distance unit configured on the TF-Luna UART outputs.
    #[arg(long, value_enum, default_value = "cm")]
    lidar_unit: DistanceUnit,

    /// Bench test: fly the precompiled hopper-sim plant with the LQR gains
    /// in GAINS_DIR and drive the engine controller (FC_EC_ADDRESS) with the
    /// same commands. Waits for `start` on stdin.
    #[cfg(feature = "sim-plant")]
    #[arg(long, value_name = "GAINS_DIR")]
    sim_plant: Option<PathBuf>,

    /// Bench config JSON for --sim-plant: caps, EDF/RCS thrust, ESC map,
    /// PA-12 geometry (default: built-in values, every cap at 70%)
    #[cfg(feature = "sim-plant")]
    #[arg(long, value_name = "FILE", requires = "sim_plant")]
    sim_config: Option<PathBuf>,

    /// Per-tick CSV log for --sim-plant (default: sim_plant_<unix time>.csv)
    #[cfg(feature = "sim-plant")]
    #[arg(long, value_name = "FILE", requires = "sim_plant")]
    sim_log: Option<PathBuf>,
}

pub fn parse() -> RuntimeConfig {
    let args = Args::parse();
    if let Err(message) = validate_lidars(&args.lidars) {
        Args::command()
            .error(ErrorKind::ValueValidation, message)
            .exit();
    }
    RuntimeConfig::from_args(args)
}

fn validate_lidars(lidars: &[LidarConfig]) -> Result<(), &'static str> {
    if !lidars.is_empty() && !(2..=4).contains(&lidars.len()) {
        return Err("configure 2–4 LiDARs using repeated --lidar options, or omit --lidar");
    }
    let mut ids = HashSet::new();
    let mut endpoints = HashSet::new();
    for lidar in lidars {
        if !ids.insert(lidar.id) {
            return Err("each LiDAR must have a unique ID");
        }
        if !endpoints.insert(lidar.endpoint) {
            return Err("each LiDAR must have a unique bridge endpoint");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_two_to_four_lidars_with_stable_ids_and_optional_units() {
        let mut args = vec![
            "flight-computer",
            "--lidar-unit",
            "mm",
            "--lidar",
            "2=udp://0.0.0.0:8002",
            "--lidar",
            "4=tcp://192.0.2.1:8004",
        ];
        for (count, next) in [
            (2, "1=udp://0.0.0.0:8001"),
            (3, "3=udp://0.0.0.0:8003"),
            (4, ""),
        ] {
            let parsed = Args::try_parse_from(&args).unwrap();
            assert_eq!(parsed.lidars.len(), count);
            assert_eq!(
                parsed
                    .lidars
                    .iter()
                    .map(|sensor| sensor.id)
                    .take(2)
                    .collect::<Vec<_>>(),
                [2, 4]
            );
            assert!(validate_lidars(&parsed.lidars).is_ok());
            assert!(matches!(parsed.lidar_unit, DistanceUnit::Mm));
            args.extend(["--lidar", next]);
        }

        let defaults = Args::parse_from(["flight-computer"]);
        assert!(validate_lidars(&defaults.lidars).is_ok());
        assert!(matches!(defaults.lidar_unit, DistanceUnit::Cm));
    }

    #[test]
    fn rejects_invalid_and_duplicate_lidar_configuration() {
        let parse = |values: &[&str]| {
            let args = std::iter::once("flight-computer")
                .chain(values.iter().flat_map(|&value| ["--lidar", value]));
            Args::try_parse_from(args).unwrap().lidars
        };

        assert!(validate_lidars(&parse(&["1=udp://0.0.0.0:8001"])).is_err());
        assert!(
            validate_lidars(&parse(&["1=udp://0.0.0.0:8001", "1=tcp://192.0.2.1:8001"])).is_err()
        );
        assert!(
            validate_lidars(&parse(&["1=udp://0.0.0.0:8001", "2=udp://0.0.0.0:8001"])).is_err()
        );
        assert!(validate_lidars(&parse(&[
            "1=udp://0.0.0.0:8001",
            "2=udp://0.0.0.0:8002",
            "3=udp://0.0.0.0:8003",
            "4=udp://0.0.0.0:8004",
            "4=udp://0.0.0.0:8005"
        ]))
        .is_err());
        for invalid in [
            "5=udp://0.0.0.0:8001",
            "1=udp://0.0.0.0:0",
            "2=tcp://0.0.0.0:8002",
            "1=serial:///dev/ttyS1",
        ] {
            assert!(Args::try_parse_from(["flight-computer", "--lidar", invalid]).is_err());
        }
    }

    #[test]
    fn parses_stacked_sensor_disable_commands() {
        let config = RuntimeConfig::from_args(Args::parse_from([
            "flight-computer",
            "disable-imu",
            "disable-magnetometer",
        ]));

        assert!(!config.worker_config.imu_enabled());
        assert!(!config.worker_config.magnetometer_enabled());
        assert!(config.worker_config.barometer_enabled());
        assert!(!config.worker_config.desktop_mode());
        assert!(config.worker_config.gps_reco_enabled());
        assert!(config.worker_config.mag_bar_enabled());
    }

    #[test]
    fn desktop_mode_disables_all_local_sensors() {
        let config = RuntimeConfig::from_args(Args::parse_from(["flight-computer", "desktop"]));

        assert!(config.worker_config.desktop_mode());
        assert!(!config.worker_config.gps_reco_enabled());
        assert!(!config.worker_config.imu_enabled());
        assert!(!config.worker_config.magnetometer_enabled());
        assert!(!config.worker_config.barometer_enabled());
        assert!(!config.worker_config.mag_bar_enabled());
    }

    #[test]
    fn disable_gps_only_turns_off_gps_worker() {
        let config =
            RuntimeConfig::from_args(Args::parse_from(["flight-computer", "disable-gps-reco"]));

        assert!(!config.worker_config.desktop_mode());
        assert!(!config.worker_config.gps_reco_enabled());
        assert!(config.worker_config.imu_enabled());
        assert!(config.worker_config.magnetometer_enabled());
        assert!(config.worker_config.barometer_enabled());
    }

    #[test]
    fn desktop_overrides_other_runtime_commands() {
        let config = RuntimeConfig::from_args(Args::parse_from([
            "flight-computer",
            "disable-gps-reco",
            "desktop",
            "disable-barometer",
        ]));

        assert!(config.worker_config.desktop_mode());
        assert!(!config.worker_config.gps_reco_enabled());
        assert!(!config.worker_config.imu_enabled());
        assert!(!config.worker_config.magnetometer_enabled());
        assert!(!config.worker_config.barometer_enabled());
    }
}
