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

/// Possible values of a GPIO pin
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum PinValue {
  /// Pin is low
  Low = 0,
  /// Pin is high
  High = 1,
}

/// Possible modes of a GPIO pin
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum PinMode {
  /// Output mode
  Output,
  /// Input mode
  Input,
}

/// All GPIO pin implementations must be safe to send between threads.
/// Abstraction for what functionality a GPIO pin should have
/// We make this trait sendable so that pins created in the caller thread
/// can be passed to the called thread.
pub trait GpioPin: Send {
  /// Configure the pin as an input or output
  fn mode(&mut self, mode: PinMode);

  /// Drive the pin high or low
  fn digital_write(&mut self, value: PinValue);

  /// Read the current logic level of the pin
  fn digital_read(&self) -> PinValue;
}

/// GPIO controller implemention for the Beaglebone
pub struct Gpio {
  fd: c_int,
  base: Mutex<*mut c_void>,
  direction: Mutex<*mut u32>,
  dataout: Mutex<*mut u32>,
  datain: Mutex<*const u32>,
}

unsafe impl Sync for Gpio {}
unsafe impl Send for Gpio {}

/// Pin implemention for the Beaglebone
pub struct Pin {
  gpio: &'static Gpio,
  index: usize,
}

/// Implementation for a Beaglebone GPIO pin 
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

/// GPIO controller implemention for the Raspberry PI.
/// Mutex is needed because RpiGpio is NOT thread safe.
pub struct RpiGpioController(Mutex<RpiGpio>);

impl RpiGpioController {
  /// Opens the Raspberry Pi GPIO controller
  pub fn open_controller() -> Result<Self, rppal::gpio::Error> {
    RpiGpio::new().map(Mutex::new).map(RpiGpioController)
  }

  /// Returns a pin for the given BCM pin number. The pin is unconfigured until
  /// you call `mode()`. The returned pin borrows this controller.
  /// Returns an RpiPin for the given BCM pin number
  pub fn get_pin(&self, pin_num: u8) -> RpiPin<'_> {
    RpiPin {
      controller: self,
      pin_num,
      inner: RpiPinInner::Unconfigured,
      last_output: PinValue::Low,
    }
  }
}

/// A single Raspberry Pi GPIO pin.
/// We state that an RpiPin CANNOT live longer than the GPIO controller that 
/// controls it.
pub struct RpiPin<'a> {
  /// GPIO controller that controls the pin
  controller: &'a RpiGpioController,
  /// BCM pin number
  pin_num: u8,
  /// Actual pin object
  inner: RpiPinInner,
  /// Last output value
  last_output: PinValue,
}

/// Inner state of the pin, which gives us access to the actual pin object
enum RpiPinInner {
  Unconfigured,
  Input(RpiInputPin),
  Output(RpiOutputPin),
}

impl<'a> GpioPin for RpiPin<'a> {
  fn mode(&mut self, mode: PinMode) {
    // access the GPIO controller and get access to the pin
    let pin = self.controller.0.lock().unwrap().get(self.pin_num)
      .expect("Failed to get Raspberry Pi GPIO pin");
    self.inner = match mode {
      PinMode::Output => {
        // configure the pin as an output pin
        let mut out = pin.into_output();
        // ensures that when pin goes out scope it stays as commanded until 
        // commanded again
        out.set_reset_on_drop(false);
        RpiPinInner::Output(out)
      }
      PinMode::Input => {
        // configure the pin as an input pin
        let mut input = pin.into_input();
        // ensures that when pin goes out scope it stays as commanded until 
        // commanded again
        input.set_reset_on_drop(false);
        RpiPinInner::Input(input)
      }
    };
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
      // TODO: i would like to get rid of last output and throw an error
      // if the pin is not an input and we call this function, but this requires
      // changing the interface of the trait and i am not sure if that is worth it
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
  /// Opens the GPIO controller, which controls a certain GPIO bank,
  /// for the given controller index.
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

  /// Returns a pin for the given index
  pub fn get_pin(&'static self, index: usize) -> Pin {
    Pin { gpio: self, index }
  }
}

impl Pin {
  /// Configure the pin as either an input or output pin
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

  /// Drive the pin high or low
  pub fn digital_write(&mut self, value: PinValue) {
    let dataout = *self.gpio.dataout.lock().unwrap();
    let mut dataout_bits = unsafe { read_volatile(dataout) };

    dataout_bits = match value {
      PinValue::Low => dataout_bits & !(1 << self.index),
      PinValue::High => dataout_bits | (1 << self.index),
    };

    unsafe { write_volatile(dataout, dataout_bits) };
  }

  /// Read the current logic level of the pin
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
