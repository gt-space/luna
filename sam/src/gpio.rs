// We are using a memory mapped implementation to increase gpio switching
// frequency
//
// https://kilobaser.com/beaglebone-black-gpios/
// The AM335x has four built-in GPIO controllers, named gpio0[], gpio1[],
// gpio2[] and gpio3[]. For each controller, there is one page of memory which
// controls each gpio controller. Each controller is responsible for 32 GPIOs.
// Each 32bit word has a specific function. Like pin configuration, controlling
// or setting a specific pin-state. Each bit in each of these words controls a
// GPIO pin. Choose function by choosing the word, choose GPIO by choosing the
// bit.
//
// https://kilobaser.com/wp-content/uploads/2021/02/BBB_SRM.pdf
// Table 12 and 13 were used to determine the P[8/9]_pin_number on expansion
// header -> gpio controller value in chip

use libc::{c_int, c_void, off_t, size_t};
use std::{
  ffi::CString,
  sync::{Arc, Mutex},
};

const GPIO_BASE_REGISTERS: [off_t; 4] =
  [0x44E0_7000, 0x4804_C000, 0x481A_C000, 0x481A_E000];
const GPIO_REGISTER_SIZE: size_t = 0xFFF;

const GPIO_OE_REGISTER: isize = 0x134;
const GPIO_DATAOUT_REGISTER: isize = 0x13C;
const GPIO_DATAIN_REGISTER: isize = 0x138;

#[derive(Debug, PartialEq)]
pub enum PinValue {
  Low = 0,
  High = 1,
}

#[derive(Debug)]
pub enum PinMode {
  Output,
  Input,
}

#[derive(Debug)]
pub enum BitOrder {
  LSBFirst,
  MSBFirst,
}

pub struct Gpio {
  fd: c_int,
  base: Mutex<*mut c_void>,
  oe: Mutex<*mut u32>,
  dataout: Mutex<*mut u32>,
  datain: *const u32,
}

unsafe impl Send for Gpio {}
unsafe impl Sync for Gpio {}

pub struct Pin {
  gpio: Arc<Gpio>,
  index: usize,
}

impl Drop for Gpio {
  fn drop(&mut self) {
    unsafe {
      libc::munmap(*self.base.lock().unwrap(), GPIO_REGISTER_SIZE);
      libc::close(self.fd);
    };
  }
}

impl Gpio {
  pub fn open(index: usize) -> Arc<Gpio> {
    let path = CString::new("/dev/mem").unwrap();
    let fd = unsafe { libc::open(path.as_ptr(), libc::O_RDWR) };

    if fd < 0 {
      panic!("Cannot open memory device");
    }

    let base = unsafe {
      libc::mmap(
        std::ptr::null_mut(),
        GPIO_REGISTER_SIZE,
        libc::PROT_READ | libc::PROT_WRITE,
        libc::MAP_SHARED,
        fd,
        GPIO_BASE_REGISTERS[index],
      )
    };

    if base.is_null() {
      panic!("Cannot map GPIO");
    }
    // } else if base != GPIO_BASE_REGISTERS[index] as *mut c_void {
    //     panic!("Cannot acquire GPIO at {index}. Did you call Gpio::open
    // twice?"); }

    let oe = Mutex::new(unsafe { base.offset(GPIO_OE_REGISTER) as *mut u32 });

    let dataout =
      Mutex::new(unsafe { base.offset(GPIO_DATAOUT_REGISTER) as *mut u32 });

    let datain = unsafe { base.offset(GPIO_DATAIN_REGISTER) as *mut u32 };

    let base = Mutex::new(base);

    Arc::new(Gpio {
      fd,
      base,
      oe,
      dataout,
      datain,
    })
  }

  pub fn get_pin(self: &Arc<Self>, index: usize) -> Pin {
    Pin {
      gpio: self.clone(),
      index,
    }
  }
}

impl Pin {
  pub fn mode(&self, mode: PinMode) {
    let oe = self.gpio.oe.lock().unwrap();
    let mut bits = unsafe { std::ptr::read_volatile(*oe) };

    bits = match mode {
      PinMode::Output => bits & !(1 << self.index),
      PinMode::Input => bits | (1 << self.index),
    };

    unsafe { std::ptr::write_volatile(*oe, bits) };
  }

  pub fn digital_write(&self, value: PinValue) {
    let dataout = self.gpio.dataout.lock().unwrap();
    let mut bits = unsafe { std::ptr::read_volatile(*dataout) };

    bits = match value {
      PinValue::Low => bits & !(1 << self.index),
      PinValue::High => bits | (1 << self.index),
    };

    unsafe { std::ptr::write_volatile(*dataout, bits) };
  }

  pub fn digital_read(&self) -> PinValue {
    let datain = self.gpio.datain;
    let bits = unsafe { std::ptr::read_volatile(datain) };

    if bits & (1 << self.index) != 0 {
      PinValue::High
    } else {
      PinValue::Low
    }
  }
}
