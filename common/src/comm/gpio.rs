use libc::{c_int, c_void, off_t, size_t};
use std::{
  ffi::CString,
  ptr::{read_volatile, write_volatile},
  sync::Mutex,
};

use rppal::gpio::{
  Gpio as RpiGpio, InputPin as RpiInputPin, OutputPin as RpiOutputPin, Level,
};

const GPIO_BASE_REGISTERS: [off_t; 4] =
  [0x44E0_7000, 0x4804_C000, 0x481A_C000, 0x481A_E000];
const GPIO_REGISTER_SIZE: size_t = 0xFFF;

const GPIO_OE_REGISTER: isize = 0x134;
const GPIO_DATAOUT_REGISTER: isize = 0x13C;
const GPIO_DATAIN_REGISTER: isize = 0x138;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum PinValue {
  Low = 0,
  High = 1,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum PinMode {
  Output,
  Input,
}

/// A simple abstraction over a single digital GPIO pin.
///
/// This is implemented for the existing BeagleBone-backed `Pin` type, and can
/// also be implemented by other platforms (e.g. Raspberry Pi) to allow
/// portable code that only depends on this interface.
///
/// All GPIO pin implementations must be safe to send between threads.
pub trait GpioPin: Send {
  /// Configure the pin as an input or output.
  fn mode(&mut self, mode: PinMode);

  /// Drive the pin high or low.
  fn digital_write(&mut self, value: PinValue);

  /// Read the current logic level of the pin.
  fn digital_read(&self) -> PinValue;
}

pub struct Gpio {
  fd: c_int,
  base: Mutex<*mut c_void>,
  direction: Mutex<*mut u32>,
  dataout: Mutex<*mut u32>,
  datain: Mutex<*const u32>,
}

unsafe impl Sync for Gpio {}
unsafe impl Send for Gpio {}

pub struct Pin {
  gpio: &'static Gpio,
  index: usize,
}

// For beaglebone
impl GpioPin for Pin {
  fn mode(&mut self, mode: PinMode) {
    Pin::mode(self, mode)
  }

  fn digital_write(&mut self, value: PinValue) {
    Pin::digital_write(self, value)
  }

  fn digital_read(&self) -> PinValue {
    Pin::digital_read(self)
  }
}

// Raspberry Pi implementation using rppal (Linux-only). This only handles GPIO;
// SPI is still done via `spidev` elsewhere.
pub struct RpiPin {
  pin_num: u8,
  inner: RpiPinInner,
  last_output: PinValue,
}

enum RpiPinInner {
  Unconfigured,
  Input(RpiInputPin),
  Output(RpiOutputPin),
}

impl RpiPin {
  /// Create a new Raspberry Pi GPIO pin wrapper for the given BCM pin number.
  /// ie. new(17) for GPIO 17
  /// The pin will initially be left unconfigured; you must call `mode` before
  /// reading or writing.
  pub fn new(pin_num: u8) -> Result<Self, rppal::gpio::Error> {
    // We don't actually configure the mode here to allow callers to pick
    // Input/Output via the shared `GpioPin` trait.
    Ok(Self {
      pin_num,
      inner: RpiPinInner::Unconfigured,
      last_output: PinValue::Low,
    })
  }

  fn reconfigure(&mut self, mode: PinMode) {
    let gpio =
      RpiGpio::new().expect("Failed to open Raspberry Pi GPIO controller");
    let pin = gpio
      .get(self.pin_num)
      .expect("Failed to get Raspberry Pi GPIO pin");

    self.inner = match mode {
      PinMode::Output => RpiPinInner::Output(pin.into_output()),
      PinMode::Input => RpiPinInner::Input(pin.into_input()),
    };
  }
}

impl GpioPin for RpiPin {
  fn mode(&mut self, mode: PinMode) {
    self.reconfigure(mode);
  }

  fn digital_write(&mut self, value: PinValue) {
    self.last_output = value;

    if let RpiPinInner::Output(ref mut pin) = self.inner {
      match value {
        PinValue::Low => pin.set_low(),
        PinValue::High => pin.set_high(),
      }
    }
  }

  fn digital_read(&self) -> PinValue {
    match &self.inner {
      RpiPinInner::Input(pin) => match pin.read() {
        Level::Low => PinValue::Low,
        Level::High => PinValue::High,
      },
      RpiPinInner::Output(_) => self.last_output,
      RpiPinInner::Unconfigured => PinValue::Low,
    }
  }
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
  pub fn open_controller(controller_index: usize) -> Gpio {
    let path = CString::new("/dev/mem").unwrap();
    let fd = unsafe { libc::open(path.as_ptr(), libc::O_RDWR) };

    if fd < 0 {
      panic!("Cannot open memory device");
    }

    // /dev/mem accesses physical memory. mmap puts files in address space
    // but dev/mem is unqiue because contents are actual hw locations

    // This comment might be wrong
    /*
    mmap creates a block of virtual memory that is mapped to the hw registers.
    Here we cannot interact with the actual register locations directly
    because there is an OS in the way so we use the virtual memory locations.
    When we pass the virtual memory location values, the OS will know how
    to map those operations to the actual register locations because of the
    relationship that was defined through the mmap call. The return value of
    mmap is a pointer to the start address of the virtual memory block.
     */

    // Place base into a mutex because it is used right after
    let base = unsafe {
      libc::mmap(
        std::ptr::null_mut(),
        GPIO_REGISTER_SIZE,
        libc::PROT_READ | libc::PROT_WRITE,
        libc::MAP_SHARED,
        fd,
        GPIO_BASE_REGISTERS[controller_index],
      )
    };

    if base.is_null() {
      panic!("Cannot map GPIO");
    } // else if base != GPIO_BASE_REGISTERS[controller_index] as *mut c_void {
      // panic!("Invalid start address for GPIO DMA operations");
      //}

    // These are all pointers to actual 32 bit wide register addresses

    let direction =
      Mutex::new(unsafe { base.offset(GPIO_OE_REGISTER) as *mut u32 });

    let dataout =
      Mutex::new(unsafe { base.offset(GPIO_DATAOUT_REGISTER) as *mut u32 });

    let datain =
      Mutex::new(unsafe { base.offset(GPIO_DATAIN_REGISTER) as *const u32 });

    Gpio {
      fd,
      base: Mutex::new(base),
      direction,
      dataout,
      datain,
    }
  }

  pub fn get_pin(&'static self, index: usize) -> Pin {
    Pin { gpio: self, index }
  }
}

impl Pin {
  pub fn mode(&mut self, mode: PinMode) {
    // gets direction, not direction dereferenced
    // lock mutex basically returns a pointer to the value it holds
    // dereference that pointer to get the actual pointer that is stored
    let direction = *self.gpio.direction.lock().unwrap();
    let mut direction_bits: u32 = unsafe { read_volatile(direction) };

    direction_bits = match mode {
      PinMode::Output => direction_bits & !(1 << self.index),
      PinMode::Input => direction_bits | (1 << self.index),
    };

    unsafe { write_volatile(direction, direction_bits) };
  }

  pub fn digital_write(&mut self, value: PinValue) {
    let dataout = *self.gpio.dataout.lock().unwrap();
    let mut dataout_bits = unsafe { read_volatile(dataout) };

    dataout_bits = match value {
      PinValue::Low => dataout_bits & !(1 << self.index),
      PinValue::High => dataout_bits | (1 << self.index),
    };

    unsafe { write_volatile(dataout, dataout_bits) };
  }

  pub fn digital_read(&self) -> PinValue {
    let datain = *self.gpio.datain.lock().unwrap();
    let datain_bits = unsafe { read_volatile(datain) };

    if datain_bits & (1 << self.index) != 0 {
      PinValue::High
    } else {
      PinValue::Low
    }
  }
}
