// Automatically generated rust module for 'command.proto' file

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

#[derive(Debug, Default, PartialEq, Clone)]
pub struct Command {
    pub command: mcfs::command::mod_Command::OneOfcommand,
}

impl<'a> MessageRead<'a> for Command {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.command = mcfs::command::mod_Command::OneOfcommand::click_valve(r.read_message::<mcfs::command::ClickValve>(bytes)?),
                Ok(18) => msg.command = mcfs::command::mod_Command::OneOfcommand::set_led(r.read_message::<mcfs::command::SetLED>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for Command {
    fn get_size(&self) -> usize {
        0
        + match self.command {
            mcfs::command::mod_Command::OneOfcommand::click_valve(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::set_led(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        match self.command {            mcfs::command::mod_Command::OneOfcommand::click_valve(ref m) => { w.write_with_tag(10, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::set_led(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::None => {},
    }        Ok(())
    }
}

pub mod mod_Command {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfcommand {
    click_valve(mcfs::command::ClickValve),
    set_led(mcfs::command::SetLED),
    None,
}

impl Default for OneOfcommand {
    fn default() -> Self {
        OneOfcommand::None
    }
}

}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct ClickValve {
    pub valve: Option<mcfs::board::ChannelIdentifier>,
    pub state: mcfs::board::ValveState,
}

impl<'a> MessageRead<'a> for ClickValve {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.valve = Some(r.read_message::<mcfs::board::ChannelIdentifier>(bytes)?),
                Ok(16) => msg.state = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for ClickValve {
    fn get_size(&self) -> usize {
        0
        + self.valve.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.state == mcfs::board::ValveState::VALVE_OPEN { 0 } else { 1 + sizeof_varint(*(&self.state) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.valve { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.state != mcfs::board::ValveState::VALVE_OPEN { w.write_with_tag(16, |w| w.write_enum(*&self.state as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct SetLED {
    pub led: Option<mcfs::board::ChannelIdentifier>,
    pub state: mcfs::board::LEDState,
}

impl<'a> MessageRead<'a> for SetLED {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.led = Some(r.read_message::<mcfs::board::ChannelIdentifier>(bytes)?),
                Ok(16) => msg.state = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for SetLED {
    fn get_size(&self) -> usize {
        0
        + self.led.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.state == mcfs::board::LEDState::LED_OFF { 0 } else { 1 + sizeof_varint(*(&self.state) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.led { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.state != mcfs::board::LEDState::LED_OFF { w.write_with_tag(16, |w| w.write_enum(*&self.state as i32))?; }
        Ok(())
    }
}

