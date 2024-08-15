use core::time;
use std::{fs::File, io::Write, thread};

pub fn set_gpio(gpio: &str) {
  println!("{}", gpio);
  let mut file: File = std::fs::OpenOptions::new()
    .write(true)
    .truncate(true)
    .open("/sys/class/gpio/export")
    .unwrap();

  file.write_all(gpio.as_bytes()).expect("Write failed");
  file.flush().expect("Flush failed");
  thread::sleep(time::Duration::from_millis(10));
}

pub fn set_output(gpio: &str) {
  println!("{}", gpio);
  let filepath = format!("/sys/class/gpio/gpio{}/direction", gpio);
  let mut file: File = std::fs::OpenOptions::new()
    .write(true)
    .truncate(true)
    .open(filepath)
    .unwrap();
  file.write_all(b"out").expect("Write failed");
  thread::sleep(time::Duration::from_millis(10));
}

pub fn set_input(gpio: &str) {
  let filepath = format!("/sys/class/gpio/gpio{}/direction", gpio);
  let mut file: File = std::fs::OpenOptions::new()
    .write(true)
    .truncate(true)
    .open(filepath)
    .unwrap();
  file.write_all(b"in").expect("Write failed");
  thread::sleep(time::Duration::from_millis(10));
}

pub fn set_high(gpio: &str) {
  let filepath = format!("/sys/class/gpio/gpio{}/value", gpio);
  let mut file: File = std::fs::OpenOptions::new()
    .write(true)
    .truncate(true)
    .open(filepath)
    .unwrap();
  file.write_all(b"1").expect("Write failed");
  thread::sleep(time::Duration::from_millis(10));
}

pub fn set_low(gpio: &str) {
  let filepath = format!("/sys/class/gpio/gpio{}/value", gpio);
  let mut file: File = std::fs::OpenOptions::new()
    .write(true)
    .truncate(true)
    .open(filepath)
    .unwrap();
  file.write_all(b"0").expect("Write failed");
  thread::sleep(time::Duration::from_millis(10));
}
