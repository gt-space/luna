use ahrs::Ahrs;
use bms::Bms;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::{any::Any, collections::HashMap, fmt, hash::Hash, time::Duration};
use serde_with::{serde_as, DurationSeconds};
use rkyv;
use bytecheck;
use core::fmt::Debug;
use std::io;

#[cfg(feature = "rusqlite")]
use rusqlite::{
  types::{FromSql, FromSqlError, FromSqlResult, ToSqlOutput, ValueRef},
  ToSql,
};

/// Deals with all communication regarding System Actuator Machines (SAMs)
pub mod sam;

/// Deals with all communication regarding the Battery Management System (BMS)
pub mod bms;

/// Deals with all communication regarding the Flight Computer (FC)
pub mod flight;

/// Deals with all communication regarding AHRS (i forgot the acronym)
pub mod ahrs;

mod gui;
pub use gui::*;

pub use crate::comm::flight::ValveSafeState;


#[cfg(feature = "gpio")]
use crate::comm::gpio::{Pin, PinMode, PinValue};

#[cfg(feature = "gpio")]
pub mod gpio;

impl fmt::Display for sam::Unit {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{}",
      match self {
        Self::Amps => "A",
        Self::Psi => "psi",
        Self::Kelvin => "K",
        Self::Pounds => "lbf",
        Self::Volts => "V",
      }
    )
  }
}

/// Holds a single measurement for either a sensor or valve.
///
/// This enum simply wraps two other types, `SensorMeasurement` and
/// `ValveMeasurement`. The reason to keep this in separate structs instead of
/// properties of the variants is that these values often need to passed around
/// independently in flight code, and enum variant properties are not mutable
/// without reconstructing the variant. This is annoying. Essentially, this
/// looks like bad / less readable code but is necessary, and convenience
/// constructs are provided to make code cleaner.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[serde(rename_all = "snake_case")]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Measurement {
  /// The raw value associated with the measurement.
  pub value: f64,

  /// The unit associated with the measurement.
  pub unit: sam::Unit,
}

impl fmt::Display for Measurement {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "{:.3} {}", self.value, self.unit)
  }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[archive_attr(derive(bytecheck::CheckBytes))]
/// Used by the Flight Computer for debugging data rates.
pub struct Statistics {
  /// A rolling average of some board's data rate.
  pub rolling_average: Duration,
  /// The difference in time between the last and second-to-last recieved
  /// packet.
  pub delta_time: Duration,
  /// time since last update in seconds
  pub time_since_last_update : f64,
}

/// Specifies what a valve should do
#[serde_as]
#[derive(Debug, Deserialize, PartialEq, Serialize, Eq, Copy, Clone, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct ValveAction {
  /// channel number that this type is talking about
  pub channel_num: u32,
  /// whether we want to be powered or unpowered
  pub powered: bool,
  /// amount of time we want to wait until we actuate a valve into its abort safe state. 
  /// ie. if timer = 10 secs for OMV, on an abort OMV will go to its abort safe state 10 secs after the board enters an abort state
  #[serde_as(as = "DurationSeconds<u64>")]
  pub timer: Duration,
}

#[derive(Debug, Deserialize, PartialEq, Serialize, Clone, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[archive_attr(derive(bytecheck::CheckBytes))]
/// Represents a single abort stage via its name, a condition that causes an abort in this stage, and valve "safe" states that valves will go to in an abort
pub struct AbortStage {
  /// Name of the abort stage 
  pub name: String,

  /// Condition that, if met, we abort.
  /// Can use the eval() in python to run strings as code
  pub abort_condition: String, 

  /// Whether we have aborted in this stage yet
  pub aborted: bool,

  /// "Safe" valve states we want boards to go if an abort occurs
  pub valve_safe_states: HashMap<String, Vec<ValveAction>>,
}

/// Holds the state of the SAMs and valves using `HashMap`s which convert a
/// node's name to its state.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct VehicleState {
  /// Holds the actual and commanded states of all valves on the vehicle.
  pub valve_states: HashMap<String, CompositeValveState>,

  /// Holds the state of every device on BMS
  pub bms: Bms,

  /// Holds the state of every device on AHRS
  pub ahrs: Ahrs,

  /// Holds the latest readings of all sensors on the vehicle.
  pub sensor_readings: HashMap<String, Measurement>,

  /// Holds a HashMap from Board ID to a 2-tuple of the Rolling Average of 
  /// obtaining a data packet from the Board ID and the duration between the
  /// last recieved and second-to-last recieved packet of the Board ID.
  pub rolling: HashMap<String, Statistics>,

  /// Defines the current abort stage that we are in
  pub abort_stage: AbortStage,
}

/// Implements all fields as default except for the AbortStage field whose name becomes "default"
impl Default for VehicleState {
  fn default() -> Self {
    Self { 
      valve_states: HashMap::new(), 
      bms: Bms::default(), 
      ahrs: Ahrs::default(), 
      sensor_readings: HashMap::default(), 
      rolling: HashMap::default(), 
      abort_stage: AbortStage { 
        name: "default".to_string(), 
        abort_condition: String::new(), 
        aborted: false,
        valve_safe_states: HashMap::new(), 
      } 
    }
  }
}

impl VehicleState {
  /// Constructs a new, empty `VehicleState`.
  pub fn new() -> Self {
    VehicleState::default()
  }
}

/// Used in a `NodeMapping` to determine which computer the action should be
/// sent to.
#[derive(
  Clone, Copy, Debug, Deserialize, Eq, MaxSize, PartialEq, Serialize
)]
#[serde(rename_all = "snake_case")]
pub enum Computer {
  /// The flight computer
  Flight,

  /// The ground computer
  Ground,
}

#[cfg(feature = "rusqlite")]
impl ToSql for Computer {
  fn to_sql(&self) -> rusqlite::Result<ToSqlOutput<'_>> {
    // see the ChannelType ToSql comment for details
    let mut json = serde_json::to_string(&self)
      .expect("failed to serialize ChannelType into JSON");

    json.pop();
    json.remove(0);

    Ok(ToSqlOutput::Owned(rusqlite::types::Value::Text(json)))
  }
}

#[cfg(feature = "rusqlite")]
impl FromSql for Computer {
  fn column_result(value: ValueRef<'_>) -> FromSqlResult<Self> {
    if let ValueRef::Text(text) = value {
      // see the ChannelType ToSql comment for details
      let mut json = vec![b'"'];
      json.extend_from_slice(text);
      json.push(b'"');

      let channel_type = serde_json::from_slice(&json)
        .map_err(|error| FromSqlError::Other(Box::new(error)))?;

      Ok(channel_type)
    } else {
      Err(FromSqlError::InvalidType)
    }
  }
}

/// The mapping of an individual node.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct NodeMapping {
  /// The text identifier, or name, of the node.
  pub text_id: String,

  /// A string identifying an individual board, corresponding to the hostname
  /// sans ".local".
  pub board_id: String,

  /// The channel type of the node, such as "valve".
  pub sensor_type: SensorType,

  /// A number identifying which channel on the SAM board controls the node.
  pub channel: u32,

  /// Which computer controls the SAM board, "flight" or "ground".
  pub computer: Computer,

  // the optional parameters below are only needed for sensors with certain
  // channel types if you're wondering why these are not kept with the
  // ChannelType variants, that is because those variants are passed back from
  // the SAM boards with data measurements. the SAM boards have no access to
  // these factors and even if they did, it would make more sense for them to
  // just convert the measurements directly.
  //
  // tl;dr this is correct and reasonable.
  /// The maximum value reading of the sensor.
  /// This is only used for sensors with channel type CurrentLoop or
  /// DifferentialSignal.
  pub max: Option<f64>,

  /// The minimum value reading of the sensor.
  /// This is only used for sensors with channel type CurrentLoop or
  /// DifferentialSignal.
  pub min: Option<f64>,

  /// The calibrated offset of the sensor.
  /// This is only used for sensors with channel type PT.
  #[serde(default)]
  pub calibrated_offset: f64,

  /// The threshold, in Amps, at which the valve is considered powered.
  pub powered_threshold: Option<f64>,

  /// Indicator of whether the valve is normally open or normally closed.
  pub normally_closed: Option<bool>,
}

/// A sequence written in Python, used by the flight computer to execute
/// arbitrary operator code.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Sequence {
  /// The unique, human-readable name which identifies the sequence.
  ///
  /// If the name is "abort" specifically, the sequence should be stored by the
  /// recipient and persisted across a machine power-down instead of run
  /// immediately.
  pub name: String,

  /// The script run immediately (except abort) upon being received.
  pub script: String,
}

/// A trigger with a
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Trigger {
  /// The unique, human-readable name which identifies the trigger.
  pub name: String,

  /// The condition upon which the trigger script is run, written in Python.
  pub condition: String,

  /// The script run when the condition is met, written in Python.
  pub script: String,

  /// Whether or not the trigger is active
  pub active: bool,
}

/// A message sent from the control server to the flight computer.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum FlightControlMessage {
  /// A set of mappings to be applied immediately.
  Mappings(Vec<NodeMapping>),

  /// A message containing a sequence to be run immediately.
  Sequence(Sequence),

  /// A trigger to be checked by the flight computer.
  Trigger(Trigger),

  /// Instructs the flight computer to stop a sequence named with the `String`
  /// parameter.
  StopSequence(String),

  /// Instructs the flight computer to execute a BMS Command on the "bms-01"
  /// board.
  BmsCommand(bms::Command),

  /// Instructs the flight computer to execute an AHRS Command on the "ahrs-01"
  /// board.
  AhrsCommand(ahrs::Command),

  /// Instructs the flight computer to run an immediate abort.
  Abort,

  /// Creates an abort stage upon confirmation the stage is valid
  AbortStageConfig(AbortStageConfig),

  /// Sets the current abort stage to an abort stage that has been created
  SetAbortStage(String),
}

/// An input config from a user
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AbortStageConfig {
  /// The unique, human-readable name which identifies the AbortStage.
  pub stage_name: String,

  /// The condition upon which the trigger script is run, written in Python.
  pub abort_condition: String,

  /// Desired safe states of valves that we want
  pub valve_safe_states: HashMap<String, ValveSafeState>,
}


// Kind of ADC
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ADCKind {
  SamRev3(SamRev3ADC),
  SamRev4Gnd(SamRev4GndADC),
  SamRev4Flight(SamRev4FlightADC),
  SamRev4FlightV2(SamRev4FlightV2ADC),
  VespulaBms(VespulaBmsADC),
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SamRev3ADC {
  CurrentLoopPt,
  DiffSensors,
  IValve,
  VValve,
  VPower,
  IPower,
  Tc1,
  Tc2,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SamRev4GndADC {
  CurrentLoopPt,
  DiffSensors,
  IValve,
  VValve,
  Rtd1,
  Rtd2,
  Rtd3,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SamRev4FlightADC {
  CurrentLoopPt,
  DiffSensors,
  IValve,
  VValve,
  Rtd1,
  Rtd2,
  Rtd3,
}

/// FSAM Rev4 2.0 ADC sensor types
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SamRev4FlightV2ADC {
  /// CurrentLoop
  CurrentLoopPt,
  /// Differential Sensors
  DiffSensors,
  /// Valve current
  IValve,
  /// Valve voltage
  VValve,
  /// Rtd 1
  Rtd1,
  /// Rtd 2
  Rtd2,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum VespulaBmsADC {
  VBatUmbCharge,
  SamAnd5V,
}

#[cfg(feature = "gpio")]
#[derive(Debug)]
pub enum ADCError {
  InvalidPositiveInputMux,
  InvalidNegativeInputMux,
  SamePositiveNegativeInputMux,
  InvalidPGAGain,
  InvalidProgrammableConversionDelay,
  InvalidDataRate,
  InvalidIDACMag,
  InvalidIDAC1Mux,
  InvalidIDAC2Mux,
  SameIDAC1IDAC2Mux,
  InvalidInternalTempSensePGAGain,
  InvalidChannel,
  InvalidGpioNum,
  WritingToGpioInput,
  OutOfBoundsRegisterRead,
  ForbiddenRegisterWrite,
  SPI(io::Error),
}

#[cfg(feature = "gpio")]
impl From<io::Error> for ADCError {
  fn from(err: io::Error) -> ADCError {
    ADCError::SPI(err)
  }
}

/// All types of ADCs (currently ads114s06 and ads124s06) implement this so that data stuctures
/// that must dynamically choose one of them to contain can do so at runtime. 
#[cfg(feature = "gpio")]
pub trait ADCFamily: Any { 
    /// creation
    fn new(
        bus: &str,
        drdy_pin: Option<Pin>,
        cs_pin: Option<Pin>,
        kind: ADCKind,
    ) -> Result<Self, ADCError>
    where
        Self: Sized;

    fn kind(&self) -> ADCKind;

    /// enable, disable CS and drdy
    fn enable_chip_select(&mut self);
    fn disable_chip_select(&mut self);
    fn check_drdy(&self) -> Option<PinValue>;

    /// SPI general control commands
    fn spi_no_operation(&mut self) -> Result<(), ADCError>;
    fn spi_wake_up_from_pwr_down_mode(&mut self) -> Result<(), ADCError>;
    fn spi_enter_pwr_down_mode(&mut self) -> Result<(), ADCError>;
    fn spi_reset(&mut self) -> Result<(), ADCError>;
    fn spi_start_conversion(&mut self) -> Result<(), ADCError>;
    fn spi_stop_conversion(&mut self) -> Result<(), ADCError>;
    fn spi_write_reg(&mut self, reg: usize, data: u8) -> Result<(), ADCError>;

    /// Read data
    fn read_counts(&mut self) -> Result<i32, ADCError>;

   /// Manipulate data
   fn calc_diff_measurement(&self, code: i32) -> f64;
   fn calc_diff_measurement_offset(&self, code: i32) -> f64;
   fn calc_four_wire_rtd_resistance(&self, code: i32, ref_resistance: f64) -> f64;

    /// SPI read reg commands
    fn spi_read_all_regs(&mut self) -> Result<[u8; 18], ADCError>;
    fn spi_read_reg(&mut self, reg: usize) -> Result<u8, ADCError>; 

    // getters for registers
    fn get_id_reg(&self) -> u8;
    fn get_status_reg(&mut self) -> Result<u8, ADCError>;
    fn get_inpmux_reg(&self) -> u8;
    fn get_pga_reg(&self) -> u8;
    fn get_datarate_reg(&self) -> u8;
    fn get_ref_reg(&self) -> u8;
    fn get_idacmag_reg(&self) -> u8;
    fn get_idacmux_reg(&self) -> u8;
    fn get_vbias_reg(&self) -> u8;
    fn get_sys_reg(&self) -> u8;
    fn get_reserved0_reg(&self) -> u8;
    fn get_ofcal0_reg(&self) -> u8;
    fn get_ofcal1_reg(&self) -> u8;
    fn get_reserved1_reg(&self) -> u8;
    fn get_fscal0_reg(&self) -> u8;
    fn get_fscal1_reg(&self) -> u8;
    fn get_gpiodat_reg(&mut self) -> Result<u8, ADCError>;
    fn get_gpiocon_reg(&self) -> u8;

    // input channel muxing
    fn get_positive_input_channel(&self) -> u8;
    fn get_negative_input_channel(&self) -> u8;
    fn set_positive_input_channel(&mut self, channel: u8) -> Result<(), ADCError>;
    fn set_negative_input_channel(&mut self, channel: u8) -> Result<(), ADCError>;
    fn set_negative_input_channel_to_aincom(&mut self) -> Result<(), ADCError>;

    // gain commanding
    fn enable_pga(&mut self) -> Result<(), ADCError>;
    fn disable_pga(&mut self) -> Result<(), ADCError>;
    fn set_pga_gain(&mut self, gain: u8) -> Result<(), ADCError>;
    fn get_pga_gain(&self) -> u8;
    fn set_programmable_conversion_delay(&mut self, delay: u16) -> Result<(), ADCError>;
    fn get_programmable_conversion_delay(&self) -> Result<u16, ADCError>;

    // data rates
    fn enable_global_chop(&mut self) -> Result<(), ADCError>;
    fn disable_global_chop(&mut self) -> Result<(), ADCError>;
    fn enable_internal_clock_disable_external(&mut self) -> Result<(), ADCError>;
    fn enable_external_clock_disable_internal(&mut self) -> Result<(), ADCError>;
    fn enable_continious_conversion_mode(&mut self) -> Result<(), ADCError>;
    fn enable_single_shot_conversion_mode(&mut self) -> Result<(), ADCError>;
    fn enable_sinc_filter(&mut self) -> Result<(), ADCError>;
    fn enable_low_latency_filter(&mut self) -> Result<(), ADCError>;
    fn set_data_rate(&mut self, rate: f64) -> Result<(), ADCError>;
    fn get_data_rate(&self) -> Result<f64, ADCError>;

    // ref
    fn disable_reference_monitor(&mut self) -> Result<(), ADCError>;
    fn enable_positive_reference_buffer(&mut self) -> Result<(), ADCError>;
    fn disable_positive_reference_buffer(&mut self) -> Result<(), ADCError>;
    fn enable_negative_reference_buffer(&mut self) -> Result<(), ADCError>;
    fn disable_negative_reference_buffer(&mut self) -> Result<(), ADCError>;
    fn set_ref_input_ref0(&mut self) -> Result<(), ADCError>;
    fn set_ref_input_ref1(&mut self) -> Result<(), ADCError>;
    fn set_ref_input_internal_2v5_ref(&mut self) -> Result<(), ADCError>;
    fn disable_internal_voltage_reference(&mut self) -> Result<(), ADCError>;
    fn enable_internal_voltage_reference_off_pwr_down(&mut self) -> Result<(), ADCError>;
    fn enable_internal_voltage_reference_on_pwr_down(&mut self) -> Result<(), ADCError>;

    // idac
    fn disable_pga_output_monitoring(&mut self) -> Result<(), ADCError>;
    fn open_low_side_pwr_switch(&mut self) -> Result<(), ADCError>;
    fn close_low_side_pwr_switch(&mut self) -> Result<(), ADCError>;
    fn set_idac_magnitude(&mut self, mag: u16) -> Result<(), ADCError>;
    fn get_idac_magnitude(&self) -> u16;

    fn enable_idac1_output_channel(&mut self, channel: u8) -> Result<(), ADCError>;
    fn enable_idac2_output_channel(&mut self, channel: u8) -> Result<(), ADCError>;
    fn disable_idac1(&mut self) -> Result<(), ADCError>;
    fn disable_idac2(&mut self) -> Result<(), ADCError>;
    fn get_idac1_output_channel(&self) -> u8;
    fn get_idac2_output_channel(&self) -> u8;

    // vbias
    fn disable_vbias(&mut self) -> Result<(), ADCError>;

    // system
    fn enable_internal_temp_sensor(&mut self, pga_gain: u8) -> Result<(), ADCError>;
    fn disable_system_monitoring(&mut self) -> Result<(), ADCError>;
    fn disable_spi_timeout(&mut self) -> Result<(), ADCError>;
    fn disable_crc_byte(&mut self) -> Result<(), ADCError>;
    fn disable_status_byte(&mut self) -> Result<(), ADCError>;

    // gpio 
    fn set_gpio_mode(&mut self, pin: u8, mode: PinMode) -> Result<(), ADCError>;
    fn get_gpio_mode(&self, pin: u8) -> Result<PinMode, ADCError>;
    fn gpio_digital_write(&mut self, pin: u8, val: PinValue) -> Result<(), ADCError>;
    fn gpio_digital_read(&mut self, pin: u8) -> Result<PinValue, ADCError>;
    fn config_gpio_as_gpio(&mut self, pin: u8) -> Result<(), ADCError>;
    fn config_gpio_as_analog_input(&mut self, pin: u8) -> Result<(), ADCError>;

    // downcasting
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

