use common::comm::gpio::{PinMode, PinValue};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct MockPin {
    pub mode: Arc<Mutex<Option<String>>>,
    pub value: Arc<Mutex<Option<bool>>>,
}

impl MockPin {
    pub fn new() -> Self {
        Self {
            mode: Arc::new(Mutex::new(None)),
            value: Arc::new(Mutex::new(None)),
        }
    }

    pub fn mode(&mut self, mode: PinMode) {
        *self.mode.lock().unwrap() = Some(format!("{:?}", mode));
    }

    pub fn digital_write(&mut self, val: PinValue) {
        *self.value.lock().unwrap() = Some(val == PinValue::High);
    }

}

#[derive(Clone)]
pub struct MockController;

impl MockController {
    pub fn get_pin(&self, _pin_num: usize) -> MockPin {
        MockPin::new()
    }
}
