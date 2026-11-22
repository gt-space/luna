use std::path::PathBuf;

use clap::{Parser, ValueEnum};

use crate::{config_readout::Section, file_logger::LoggerConfig};

/// Runtime commands for the flight computer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum SensorCommands {
    /// Disable all sensor workers, including the Ethernet LiDARs
    Desktop,
    /// Disable the GPS/RECO worker
    DisableGpsReco,
    /// Disable the FC-local IMU
    DisableImu,
    /// Disable the FC-local magnetometer
    DisableMagnetometer,
    /// Disable the FC-local barometer
    DisableBarometer,
    /// Disable both Ethernet LiDARs
    DisableLidar,
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
    lidar_enabled: bool,
}

impl Default for WorkerConfig {
    fn default() -> Self {
        Self {
            desktop_mode: false,
            gps_reco_enabled: true,
            imu_enabled: true,
            magnetometer_enabled: true,
            barometer_enabled: true,
            lidar_enabled: true,
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
                    plan.lidar_enabled = false;
                }
                SensorCommands::DisableGpsReco => plan.gps_reco_enabled = false,
                SensorCommands::DisableImu => plan.imu_enabled = false,
                SensorCommands::DisableMagnetometer => plan.magnetometer_enabled = false,
                SensorCommands::DisableBarometer => plan.barometer_enabled = false,
                SensorCommands::DisableLidar => plan.lidar_enabled = false,
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

    /// Whether the flight computer should connect to both LiDAR adapters.
    pub fn lidar_enabled(&self) -> bool {
        self.lidar_enabled
    }
}

#[derive(Debug)]
pub struct RuntimeConfig {
    pub worker_config: WorkerConfig,
    pub logger_config: LoggerConfig,
    pub print_gps: bool,
    pub show_config: Option<Vec<Section>>,
    /// `--shadow-nav`: run the GMEKF on the sensors and log it, without
    /// feeding control
    pub shadow_nav: bool,
    /// `--shadow-nav-config`: JSON settings for the shadow filter
    pub shadow_nav_config: Option<PathBuf>,
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
            shadow_nav: args.shadow_nav || args.shadow_nav_config.is_some(),
            shadow_nav_config: args.shadow_nav_config,
            #[cfg(feature = "sim-plant")]
            sim_plant: args
                .sim_plant
                .map(|gains| crate::sim_plant::SimPlantConfig {
                    gains,
                    config: args.sim_config,
                    log: args.sim_log,
                    gains_controller: args.sim_gains_controller,
                    allow_dirty: args.sim_allow_dirty,
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

    /// Run the GMEKF navigation filter in shadow mode: it reads the IMU,
    /// barometer and GPS and logs its estimate to the first log directory,
    /// but never feeds control (disabled by default)
    #[arg(long, default_value_t = false)]
    shadow_nav: bool,

    /// JSON settings for --shadow-nav (IMU-to-body rotation, GPS lever arm,
    /// noise); implies --shadow-nav
    #[arg(long, value_name = "FILE")]
    shadow_nav_config: Option<PathBuf>,

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

    /// Controller the --sim-plant gains must have been exported for
    #[cfg(feature = "sim-plant")]
    #[arg(
        long,
        value_name = "NAME",
        requires = "sim_plant",
        default_value = "lqr_ctv_tuning"
    )]
    sim_gains_controller: String,

    /// Accept --sim-plant gains or a plant built from a modified (-dirty)
    /// hopper tree
    #[cfg(feature = "sim-plant")]
    #[arg(long, requires = "sim_plant")]
    sim_allow_dirty: bool,
}

pub fn parse() -> RuntimeConfig {
    RuntimeConfig::from_args(Args::parse())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "sim-plant")]
    #[test]
    fn sim_plant_gains_must_be_for_the_tuned_ctv_controller_and_a_clean_commit() {
        let config = RuntimeConfig::from_args(Args::parse_from([
            "flight-computer",
            "--sim-plant",
            "gains",
        ]));
        let sim = config.sim_plant.unwrap();

        assert_eq!(sim.gains_controller, "lqr_ctv_tuning");
        assert!(!sim.allow_dirty);

        let config = RuntimeConfig::from_args(Args::parse_from([
            "flight-computer",
            "--sim-plant",
            "gains",
            "--sim-gains-controller",
            "lqr_sched",
            "--sim-allow-dirty",
        ]));
        let sim = config.sim_plant.unwrap();

        assert_eq!(sim.gains_controller, "lqr_sched");
        assert!(sim.allow_dirty);
    }

    #[test]
    fn lidar_workers_are_enabled_by_default() {
        let config = RuntimeConfig::from_args(Args::parse_from(["flight-computer"]));

        assert!(config.worker_config.lidar_enabled());
    }

    #[test]
    fn disable_lidar_stacks_with_other_sensor_commands() {
        let config = RuntimeConfig::from_args(Args::parse_from([
            "flight-computer",
            "disable-lidar",
            "disable-imu",
        ]));

        assert!(!config.worker_config.lidar_enabled());
        assert!(!config.worker_config.imu_enabled());
        assert!(config.worker_config.gps_reco_enabled());
        assert!(config.worker_config.mag_bar_enabled());
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
        assert!(!config.worker_config.lidar_enabled());
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
