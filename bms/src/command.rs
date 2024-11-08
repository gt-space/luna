use common::comm::{gpio::{Gpio, PinMode::Output, PinValue::{Low, High}}, bms::Command};
use std::{thread, time::Duration};
use common::comm::{Gpio, Pin, PinMode::Output, PinValue::{Low, High}, ADCKind, SamControlMessage};
use once_cell::sync::Lazy;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());

// controller = floor(GPIO#/32)
// pin = remainder

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn init_gpio() {
  // set battery enable low
  // set sam enable low (disable)
  // set charge enable low (disable)
  // set estop reset low
  command::disable_battery_power();
  command::disable_sam_power();
  command::disable_charger();
  command::estop_init();

  for chip_select_pin in get_cs_mappings().values_mut() {
    chip_select_pin.digital_write(High); // active low
  }
}

pub fn get_cs_mappings() -> HashMap<ADCKind, Pin> {
  let mut vbat_umb_charge_chip_select: Pin = GPIO_CONTROLLERS[0].get_pin(30);
  vbat_umb_charge_chip_select.mode(Output);
  let mut sam_and_5v_chip_select: Pin = GPIO_CONTROLLERS[0].get_pin(31);
  sam_and_5v_chip_select.mode(Output);

  HashMap::from([(ADCKind::VBatUmbCharge, vbat_umb_charge_chip_select),
  (ADCKind::SamAnd5V, sam_and_5v_chip_select)])
}

// channel = 10 : powered = True
pub fn enable_battery_power() {
  // P8 GPOI 36 Pin 69
  let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 10 : powered = False
pub fn disable_battery_power() {
  // P8 GPOI 36 Pin 69
  let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(Low);
}

// channel = 11 : powered = True
pub fn enable_sam_power() {
  // P8 GPIO22 Pin 65
  let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 11 : powered = False
pub fn disable_sam_power() {
  // P8 GPIO22 Pin 65
  let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(Low);
}

// channel = 12 : powered = True
pub fn enable_charger() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 12 : powered = False
pub fn disable_charger() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(Low);
}

// can be included in normal execution code
pub fn estop_init() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(Low);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(High);
}

// not needed rn
pub fn estop_reset() {
  // P8 GPIO 65 Pin 64
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
}

// not needed rn
pub fn set_estop_low() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(Low);
}


// no need to implement now
pub fn reco_enable(channel: u32) {
  match channel {
    1 => {
      // P8 GPIO 68 Pin 56
      let mut pin = GPIO_CONTROLLERS[2].get_pin(4);
      pin.mode(Output);
      pin.digital_write(High);
    }
    2 => {
      // P8 GPIO 67 Pin 54
      let mut pin = GPIO_CONTROLLERS[2].get_pin(3);
      pin.mode(Output);
      pin.digital_write(High);
    }
    3 => {
      // P8 GPIO 66 Pin 53
      let mut pin = GPIO_CONTROLLERS[2].get_pin(2);
      pin.mode(Output);
      pin.digital_write(High);
    }
    _ => println!("Error"),
  }
}

pub fn execute(command: Command) {
  match command {
    Command::Charge(x) => {
      if x {
        enable_charger();
      } else {
        disable_charger();
      }
    },

    Command::BatteryLoadSwitch(x) => {
      if x {
        enable_battery_power();
      } else {
        disable_battery_power();
      }
    },

    Command::SamLoadSwitch(x) => {
      if x {
        enable_battery_power();
      } else {
        disable_battery_power();
      }
    },

    Command::ResetEstop => {
      estop_reset();
    }
  }
}


// DEPRECATED!
// HOW TO ACTIVATE BMS COMMANDS:
// Mapppings settings:
// Text ID (channel) = battey_power (20), sam_power (21), charger (22)
// SensorType = Valve
// Computer = Flight
// NormallyClosed = False
// Board ID = bms-01
// HOW TO SET BMS PROPERTIES
// Open Valve = True
// Close Valve = False