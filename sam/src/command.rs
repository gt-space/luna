use common::comm::{
  gpio::{
    ControlModuleRegister,
    CONTROL_MODULE_BASE,
    CONTROL_MODULE_SIZE,
    Gpio,
    Pin,
    PinMode::Output,
    PinValue::{High, Low}
  },
  sam::SamControlMessage
};
use std::{thread, time::Duration};
use std::collections::HashMap;
use libc::{c_int, c_void, off_t, size_t};
use std::{
  ffi::CString, ptr::{read_volatile, write_volatile}, sync::Mutex
};

use crate::pins::{GPIO_CONTROLLERS, VALVE_PINS, VALVE_CURRENT_PINS, SPI_INFO, GpioInfo};
use crate::{SamVersion, SAM_VERSION};

pub fn execute(command: SamControlMessage) {
  match command {
    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    },

    SamControlMessage::Abort => {
      safe_valves();
    }
  }
}

pub fn safe_valves() {
  for i in 1..7 {
    actuate_valve(i, false); // turn off all valves
  }
}

/* So the Beaglebone is really annoying and the config-pin script is not
available to be used on every pin. Some pins have GPIO as available modes
but are initially used during boot in the LCD and GPMC modes. While I could have
attempted to modify this and make them available through device tree overlays
I instead go through /dev/mem access to directly modify the registers that
control the pins, very similar to how we use /dev/mem to toggle GPIOs and read
their states.
 */
// pub fn fix_gpio() {
//   if *SAM_VERSION == SamVersion::Rev3 {
//     // no modifications needed for rev3 hardware :)
//     return
//   }

//   // this file gives access to actual hardware memory
//   let path = CString::new("/dev/mem").unwrap();
//   let fd = unsafe { libc::open(path.as_ptr(), libc::O_RDWR) };
//   if fd < 0 {
//     panic!("Cannot open memory device");
//   }

//   /* The control module memory block has 32 bit registers for each
//   configurable pin on the AM335x processor. Within each register things such
//   as the mode, whether a pullup or pulldown resistor on the pin is selected,
//   and whether or not the pull resistor is enabled can be configured. With the
//   following mmap call, the entire control module block is accessed.
//    */
//   let control_module_ptr: *mut c_void = unsafe {
//     libc::mmap(
//       std::ptr::null_mut(),
//       CONTROL_MODULE_SIZE,
//       libc::PROT_READ | libc::PROT_WRITE,
//       libc::MAP_SHARED,
//       fd,
//       CONTROL_MODULE_BASE
//     )
//   };

//   if control_module_ptr.is_null() {
//     panic!("Cannot map Control Module memory");
//   }

//   if *SAM_VERSION == SamVersion::Rev4Ground {
//     // valve 1 modifications
//     unsafe {
//       let reg_ptr = control_module_ptr.offset(ControlModuleRegister::conf_gpmc_ad0 as isize) as *mut u32;
//       let mut bits: u32 = read_volatile(reg_ptr as *const u32);
//       bits |= 1 << 4; // enable pullup resistor
//       bits |= 1 << 3; // disable pull resistor (if it were enabled it should be pullup)
//       bits |= 7; // mode 7 is gpio
//       write_volatile(reg_ptr, bits);
//     }

//     // valve 2 modifications
//     unsafe {
//       let reg_ptr = control_module_ptr.offset(ControlModuleRegister::conf_gpmc_ad4 as isize) as *mut u32;
//       let mut bits: u32 = read_volatile(reg_ptr as *const u32);
//       bits |= 1 << 4; // enable pullup resistor
//       bits |= 1 << 3; // disable pull resistor (if it were enabled it should be pullup)
//       bits |= 7; // mode 7 is gpio
//       write_volatile(reg_ptr, bits);
//     }
//   } else if *SAM_VERSION == SamVersion::Rev4Flight {
//     // valve 6 modifications
//     unsafe {
//       let reg_ptr = control_module_ptr.offset(ControlModuleRegister::conf_lcd_data2 as isize) as *mut u32;
//       let mut bits: u32 = read_volatile(reg_ptr as *const u32);
//       bits |= 1 << 4; // enable pullup resistor
//       bits |= 1 << 3; // disable pull resistor (if it were enabled it should be pullup)
//       bits |= 7; // mode 7 is gpio
//       write_volatile(reg_ptr, bits);
//     }
//   }

//   // free the memory and free the file descriptor
//   unsafe {
//     libc::munmap(control_module_ptr, CONTROL_MODULE_SIZE);
//     libc::close(fd);
//   }
// }

pub fn init_gpio() {
  // disable all chip selects
  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin = GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low so make it high to disable
      cs_pin.digital_write(High);
    }
  }

  // handles CS for cold junction IC on rev3 (not an ADC)
  if *SAM_VERSION == SamVersion::Rev3 {
    let mut cs_tc_cjc1 = GPIO_CONTROLLERS[2].get_pin(23);
    cs_tc_cjc1.mode(Output);
    cs_tc_cjc1.digital_write(High); // chip select is active low

    let mut cs_tc_cjc2 = GPIO_CONTROLLERS[0].get_pin(7);
    cs_tc_cjc2.mode(Output);
    cs_tc_cjc2.digital_write(High); // chip select is active low
  }
  
  // turn off all valves
  actuate_valve(1, false);
  actuate_valve(2, false);
  actuate_valve(3, false);
  actuate_valve(4, false);
  actuate_valve(5, false);
  actuate_valve(6, false);

  // handle the pins that choose which valve the current feedback is from
  if *SAM_VERSION != SamVersion::Rev3 {
    for gpio_info in VALVE_CURRENT_PINS.values() {
      let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
      pin.mode(Output); // like so incredibly redundant
      pin.digital_write(Low); // start on valves 1, 3, 5
    }
  }
}

fn actuate_valve(channel: u32, powered: bool) {
  if (channel < 1 || channel > 6) {
    panic!("Invalid valve channel number")
  }

  let gpio_info = VALVE_PINS.get(&channel).unwrap();
  let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
  pin.mode(Output);

  match powered {
    true => {
      pin.digital_write(High);
    },

    false => {
      pin.digital_write(Low);
    }
  }
}