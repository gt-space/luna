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
                Ok(10) => msg.command = mcfs::command::mod_Command::OneOfcommand::data_directive(r.read_message::<mcfs::command::DataDirective>(bytes)?),
                Ok(18) => msg.command = mcfs::command::mod_Command::OneOfcommand::click_valve(r.read_message::<mcfs::command::ClickValve>(bytes)?),
                Ok(26) => msg.command = mcfs::command::mod_Command::OneOfcommand::set_led(r.read_message::<mcfs::command::SetLED>(bytes)?),
                Ok(34) => msg.command = mcfs::command::mod_Command::OneOfcommand::device_discovery(r.read_message::<mcfs::command::DeviceDiscovery>(bytes)?),
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
            mcfs::command::mod_Command::OneOfcommand::data_directive(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::click_valve(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::set_led(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::device_discovery(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::command::mod_Command::OneOfcommand::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        match self.command {            mcfs::command::mod_Command::OneOfcommand::data_directive(ref m) => { w.write_with_tag(10, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::click_valve(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::set_led(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::device_discovery(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::command::mod_Command::OneOfcommand::None => {},
    }        Ok(())
    }
}

pub mod mod_Command {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfcommand {
    data_directive(mcfs::command::DataDirective),
    click_valve(mcfs::command::ClickValve),
    set_led(mcfs::command::SetLED),
    device_discovery(mcfs::command::DeviceDiscovery),
    None,
}

impl Default for OneOfcommand {
    fn default() -> Self {
        OneOfcommand::None
    }
}

}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct DataDirective {
    pub node: Option<mcfs::device::NodeIdentifier>,
    pub level: mcfs::common::LogLevel,
    pub freq_log: f32,
    pub freq_send: f32,
}

impl<'a> MessageRead<'a> for DataDirective {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.node = Some(r.read_message::<mcfs::device::NodeIdentifier>(bytes)?),
                Ok(16) => msg.level = r.read_enum(bytes)?,
                Ok(29) => msg.freq_log = r.read_float(bytes)?,
                Ok(37) => msg.freq_send = r.read_float(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for DataDirective {
    fn get_size(&self) -> usize {
        0
        + self.node.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.level == mcfs::common::LogLevel::LOG_LEVEL_DATA { 0 } else { 1 + sizeof_varint(*(&self.level) as u64) }
        + if self.freq_log == 0f32 { 0 } else { 1 + 4 }
        + if self.freq_send == 0f32 { 0 } else { 1 + 4 }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.node { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.level != mcfs::common::LogLevel::LOG_LEVEL_DATA { w.write_with_tag(16, |w| w.write_enum(*&self.level as i32))?; }
        if self.freq_log != 0f32 { w.write_with_tag(29, |w| w.write_float(*&self.freq_log))?; }
        if self.freq_send != 0f32 { w.write_with_tag(37, |w| w.write_float(*&self.freq_send))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct ClickValve {
    pub valve: Option<mcfs::device::NodeIdentifier>,
    pub state: mcfs::device::ValveState,
}

impl<'a> MessageRead<'a> for ClickValve {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.valve = Some(r.read_message::<mcfs::device::NodeIdentifier>(bytes)?),
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
        + if self.state == mcfs::device::ValveState::VALVE_OPEN { 0 } else { 1 + sizeof_varint(*(&self.state) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.valve { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.state != mcfs::device::ValveState::VALVE_OPEN { w.write_with_tag(16, |w| w.write_enum(*&self.state as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct SetLED {
    pub led: Option<mcfs::device::NodeIdentifier>,
    pub state: mcfs::device::LEDState,
}

impl<'a> MessageRead<'a> for SetLED {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.led = Some(r.read_message::<mcfs::device::NodeIdentifier>(bytes)?),
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
        + if self.state == mcfs::device::LEDState::LED_OFF { 0 } else { 1 + sizeof_varint(*(&self.state) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.led { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.state != mcfs::device::LEDState::LED_OFF { w.write_with_tag(16, |w| w.write_enum(*&self.state as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct DeviceDiscovery {
    pub requesting_device_type: mcfs::device::DeviceType,
}

impl<'a> MessageRead<'a> for DeviceDiscovery {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.requesting_device_type = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for DeviceDiscovery {
    fn get_size(&self) -> usize {
        0
        + if self.requesting_device_type == mcfs::device::DeviceType::SERVER { 0 } else { 1 + sizeof_varint(*(&self.requesting_device_type) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.requesting_device_type != mcfs::device::DeviceType::SERVER { w.write_with_tag(8, |w| w.write_enum(*&self.requesting_device_type as i32))?; }
        Ok(())
    }
}

