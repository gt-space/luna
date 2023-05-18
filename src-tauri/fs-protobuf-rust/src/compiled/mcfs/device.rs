// Automatically generated rust module for 'device.proto' file

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(unused_imports)]
#![allow(unknown_lints)]
#![allow(clippy::all)]
#![cfg_attr(rustfmt, rustfmt_skip)]


use quick_protobuf::{MessageRead, MessageWrite, BytesReader, Writer, WriterBackend, Result};
use quick_protobuf::sizeofs::*;
use super::super::*;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Channel {
    GPIO = 0,
    LED = 1,
    RAIL_3V3 = 2,
    RAIL_5V = 3,
    RAIL_5V5 = 4,
    RAIL_24V = 5,
    CURRENT_LOOP = 6,
    DIFFERENTIAL_SIGNAL = 7,
    TEMPERATURE_DETECTOR = 8,
    VALVE = 9,
}

impl Default for Channel {
    fn default() -> Self {
        Channel::GPIO
    }
}

impl From<i32> for Channel {
    fn from(i: i32) -> Self {
        match i {
            0 => Channel::GPIO,
            1 => Channel::LED,
            2 => Channel::RAIL_3V3,
            3 => Channel::RAIL_5V,
            4 => Channel::RAIL_5V5,
            5 => Channel::RAIL_24V,
            6 => Channel::CURRENT_LOOP,
            7 => Channel::DIFFERENTIAL_SIGNAL,
            8 => Channel::TEMPERATURE_DETECTOR,
            9 => Channel::VALVE,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for Channel {
    fn from(s: &'a str) -> Self {
        match s {
            "GPIO" => Channel::GPIO,
            "LED" => Channel::LED,
            "RAIL_3V3" => Channel::RAIL_3V3,
            "RAIL_5V" => Channel::RAIL_5V,
            "RAIL_5V5" => Channel::RAIL_5V5,
            "RAIL_24V" => Channel::RAIL_24V,
            "CURRENT_LOOP" => Channel::CURRENT_LOOP,
            "DIFFERENTIAL_SIGNAL" => Channel::DIFFERENTIAL_SIGNAL,
            "TEMPERATURE_DETECTOR" => Channel::TEMPERATURE_DETECTOR,
            "VALVE" => Channel::VALVE,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ValveState {
    VALVE_OPEN = 0,
    VALVE_CLOSED = 1,
}

impl Default for ValveState {
    fn default() -> Self {
        ValveState::VALVE_OPEN
    }
}

impl From<i32> for ValveState {
    fn from(i: i32) -> Self {
        match i {
            0 => ValveState::VALVE_OPEN,
            1 => ValveState::VALVE_CLOSED,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for ValveState {
    fn from(s: &'a str) -> Self {
        match s {
            "VALVE_OPEN" => ValveState::VALVE_OPEN,
            "VALVE_CLOSED" => ValveState::VALVE_CLOSED,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum LEDState {
    LED_OFF = 0,
    LED_ON = 1,
}

impl Default for LEDState {
    fn default() -> Self {
        LEDState::LED_OFF
    }
}

impl From<i32> for LEDState {
    fn from(i: i32) -> Self {
        match i {
            0 => LEDState::LED_OFF,
            1 => LEDState::LED_ON,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for LEDState {
    fn from(s: &'a str) -> Self {
        match s {
            "LED_OFF" => LEDState::LED_OFF,
            "LED_ON" => LEDState::LED_ON,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DeviceType {
    SERVER = 0,
    FLIGHT_COMPUTER = 1,
    SAM = 2,
}

impl Default for DeviceType {
    fn default() -> Self {
        DeviceType::SERVER
    }
}

impl From<i32> for DeviceType {
    fn from(i: i32) -> Self {
        match i {
            0 => DeviceType::SERVER,
            1 => DeviceType::FLIGHT_COMPUTER,
            2 => DeviceType::SAM,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for DeviceType {
    fn from(s: &'a str) -> Self {
        match s {
            "SERVER" => DeviceType::SERVER,
            "FLIGHT_COMPUTER" => DeviceType::FLIGHT_COMPUTER,
            "SAM" => DeviceType::SAM,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct NodeIdentifier {
    pub board_id: u32,
    pub channel: mcfs::device::Channel,
    pub node_id: u32,
}

impl<'a> MessageRead<'a> for NodeIdentifier {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.board_id = r.read_uint32(bytes)?,
                Ok(16) => msg.channel = r.read_enum(bytes)?,
                Ok(24) => msg.node_id = r.read_uint32(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for NodeIdentifier {
    fn get_size(&self) -> usize {
        0
        + if self.board_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.board_id) as u64) }
        + if self.channel == mcfs::device::Channel::GPIO { 0 } else { 1 + sizeof_varint(*(&self.channel) as u64) }
        + if self.node_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.node_id) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.board_id != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.board_id))?; }
        if self.channel != mcfs::device::Channel::GPIO { w.write_with_tag(16, |w| w.write_enum(*&self.channel as i32))?; }
        if self.node_id != 0u32 { w.write_with_tag(24, |w| w.write_uint32(*&self.node_id))?; }
        Ok(())
    }
}

