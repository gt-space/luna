
use std::time;
// opening and closing of gpio pins
// search up how to toggle gpio pins on a beaglebone
// use crate core;
use std::{fs::File, io::Write, thread};

// select the gpio to which we want to set input/output to high/low
pub fn set_gpio(gpio: &str) {
    println!("{}", gpio); // select the gpio name
    let mut file: File = std::fs::OpenOptions::new() // create a new file
                                .write(true) // set the operation as write
                                .truncate(true) // truncate additional white space
                                .open("/sys/class/gpio/export") // open the correct directory
                                .unwrap(); // returns the value if it is Some, or panics if it is None
    file.write(gpio.as_bytes()).expect("Write failed"); // write  the gpio to the file
    file.flush().expect("Flush failed"); // "save" the file content
    thread::sleep(time::Duration::from_millis(10)); // give 10 millisecond buffer between commands
}

// set the output of the gpio
pub fn set_output(gpio: &str) {
    println!("{}", gpio);
    let filepath = format!("/sys/class/gpio/gpio{}/direction", gpio); // save in direction directory (or parameter)
    let mut file: File = std::fs::OpenOptions::new()
                                .write(true)
                                .truncate(true)
                                .open(filepath)
                                .unwrap();
    file.write(b"out").expect("Write failed");
    thread::sleep(time::Duration::from_millis(10));
}

// set the input of the gpio
pub fn set_input(gpio: &str) {
    println!("{}", gpio);
    let filepath = format!("/sys/class/gpio/gpio{}/direction", gpio);
    let mut file: File = std::fs::OpenOptions::new()
                                .write(true)
                                .truncate(true)
                                .open(filepath)
                                .unwrap();
    file.write(b"in").expect("Write failed");
    thread::sleep(time::Duration::from_millis(10));
}

// set the input or output (depending on user) to high (1)
pub fn set_high(gpio: &str) {
    println!("{}", gpio);
    let filepath = format!("/sys/class/gpio/gpio{}/value", gpio); // save in value directory (or parameter)
    let mut file: File = std::fs::OpenOptions::new()
                                .write(true)
                                .truncate(true)
                                .open(filepath)
                                .unwrap();
    file.write(b"1").expect("Write failed");
    thread::sleep(time::Duration::from_millis(10));
}

// set the input or output (depending on user) to low (0)
pub fn set_low(gpio: &str) {
    println!("{}", gpio);
    let filepath = format!("/sys/class/gpio/gpio{}/value", gpio);
    let mut file: File = std::fs::OpenOptions::new()
                                .write(true)
                                .truncate(true)
                                .open(filepath)
                                .unwrap();
    file.write(b"0").expect("Write failed");
    thread::sleep(time::Duration::from_millis(10));
}