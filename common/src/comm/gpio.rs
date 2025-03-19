use libc::{c_int, c_void, off_t, size_t};
use std::{
  ffi::CString,
  ptr::{read_volatile, write_volatile},
  sync::Mutex,
};

const GPIO_BASE_REGISTERS: [off_t; 4] =
  [0x44E0_7000, 0x4804_C000, 0x481A_C000, 0x481A_E000];
const GPIO_REGISTER_SIZE: size_t = 0xFFF;

const GPIO_OE_REGISTER: isize = 0x134;
const GPIO_DATAOUT_REGISTER: isize = 0x13C;
const GPIO_DATAIN_REGISTER: isize = 0x138;

const CONTROL_MODULE_BASE_REGISTERS: off_t = 0x44E1_0000;
const CONTROL_MODULE_REGISTER_SIZE: size_t = 0xFFF;

#[derive(Debug, PartialEq)]
pub enum PinValue {
  Low = 0,
  High = 1,
}

#[derive(Debug, PartialEq, Eq)]
pub enum PinMode {
  Output,
  Input,
}

pub struct Gpio {
  fd: c_int,
  base: Mutex<*mut c_void>,
  direction: Mutex<*mut u32>,
  dataout: Mutex<*mut u32>,
  datain: Mutex<*const u32>,
  control: Mutex<*mut c_void>,
}

unsafe impl Sync for Gpio {}
unsafe impl Send for Gpio {}

pub struct Pin {
  gpio: &'static Gpio,
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

    let control = unsafe {
      libc::mmap(
          std::ptr::null_mut(),
          CONTROL_MODULE_REGISTER_SIZE,
          libc::PROT_READ | libc::PROT_WRITE,
          libc::MAP_SHARED,
          fd,
          CONTROL_MODULE_BASE_REGISTERS,
      )
    };

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
      control: Mutex::new(control),
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

  pub fn enable_pullup(&self, offset: isize) {
    let control = *self.gpio.control.lock().unwrap(); // get control register
    let control_bits_ptr = unsafe { control.offset(offset) as *mut u32 };
    // pointer to location of control bits

    let mut control_bits = unsafe { read_volatile(control_bits_ptr) };
    //get current value of control bits
    // to enable pull up resistor we want bits 4-3 to be 0b10
    // so we clear bits 4-3 and then set bit 4 to 1 and 3 to 0
    control_bits = control_bits & !(0b11 << 3); // clear pullup/pulldown bits
    control_bits = control_bits | (0b10 << 3); // set pullup bit

    unsafe { write_volatile(control_bits_ptr, control_bits) };
  }
}
