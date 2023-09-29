// Automatically generated rust module for 'board.proto' file

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
pub enum ChannelType {
    GPIO = 0,
    LED = 1,
    RAIL_3V3 = 2,
    RAIL_5V = 3,
    RAIL_5V5 = 4,
    RAIL_24V = 5,
    CURRENT_LOOP = 6,
    DIFFERENTIAL_SIGNAL = 7,
    TC = 8,
    RTD = 9,
    VALVE = 10,
    VALVE_CURRENT = 11,
    VALVE_VOLTAGE = 12,
}

impl Default for ChannelType {
    fn default() -> Self {
        ChannelType::GPIO
    }
}

impl From<i32> for ChannelType {
    fn from(i: i32) -> Self {
        match i {
            0 => ChannelType::GPIO,
            1 => ChannelType::LED,
            2 => ChannelType::RAIL_3V3,
            3 => ChannelType::RAIL_5V,
            4 => ChannelType::RAIL_5V5,
            5 => ChannelType::RAIL_24V,
            6 => ChannelType::CURRENT_LOOP,
            7 => ChannelType::DIFFERENTIAL_SIGNAL,
            8 => ChannelType::TC,
            9 => ChannelType::RTD,
            10 => ChannelType::VALVE,
            11 => ChannelType::VALVE_CURRENT,
            12 => ChannelType::VALVE_VOLTAGE,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for ChannelType {
    fn from(s: &'a str) -> Self {
        match s {
            "GPIO" => ChannelType::GPIO,
            "LED" => ChannelType::LED,
            "RAIL_3V3" => ChannelType::RAIL_3V3,
            "RAIL_5V" => ChannelType::RAIL_5V,
            "RAIL_5V5" => ChannelType::RAIL_5V5,
            "RAIL_24V" => ChannelType::RAIL_24V,
            "CURRENT_LOOP" => ChannelType::CURRENT_LOOP,
            "DIFFERENTIAL_SIGNAL" => ChannelType::DIFFERENTIAL_SIGNAL,
            "TC" => ChannelType::TC,
            "RTD" => ChannelType::RTD,
            "VALVE" => ChannelType::VALVE,
            "VALVE_CURRENT" => ChannelType::VALVE_CURRENT,
            "VALVE_VOLTAGE" => ChannelType::VALVE_VOLTAGE,
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
pub enum BoardType {
    SERVER = 0,
    FLIGHT_COMPUTER = 1,
    GROUND_COMPUTER = 2,
    SAM = 3,
}

impl Default for BoardType {
    fn default() -> Self {
        BoardType::SERVER
    }
}

impl From<i32> for BoardType {
    fn from(i: i32) -> Self {
        match i {
            0 => BoardType::SERVER,
            1 => BoardType::FLIGHT_COMPUTER,
            2 => BoardType::GROUND_COMPUTER,
            3 => BoardType::SAM,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for BoardType {
    fn from(s: &'a str) -> Self {
        match s {
            "SERVER" => BoardType::SERVER,
            "FLIGHT_COMPUTER" => BoardType::FLIGHT_COMPUTER,
            "GROUND_COMPUTER" => BoardType::GROUND_COMPUTER,
            "SAM" => BoardType::SAM,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct ChannelIdentifier {
    pub board_id: u32,
    pub channel_type: mcfs::board::ChannelType,
    pub channel: u32,
}

impl<'a> MessageRead<'a> for ChannelIdentifier {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.board_id = r.read_uint32(bytes)?,
                Ok(16) => msg.channel_type = r.read_enum(bytes)?,
                Ok(24) => msg.channel = r.read_uint32(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for ChannelIdentifier {
    fn get_size(&self) -> usize {
        0
        + if self.board_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.board_id) as u64) }
        + if self.channel_type == mcfs::board::ChannelType::GPIO { 0 } else { 1 + sizeof_varint(*(&self.channel_type) as u64) }
        + if self.channel == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.channel) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.board_id != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.board_id))?; }
        if self.channel_type != mcfs::board::ChannelType::GPIO { w.write_with_tag(16, |w| w.write_enum(*&self.channel_type as i32))?; }
        if self.channel != 0u32 { w.write_with_tag(24, |w| w.write_uint32(*&self.channel))?; }
        Ok(())
    }
}

