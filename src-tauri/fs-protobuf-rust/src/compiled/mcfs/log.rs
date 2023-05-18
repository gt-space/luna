// Automatically generated rust module for 'log.proto' file

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(unused_imports)]
#![allow(unknown_lints)]
#![allow(clippy::all)]
#![cfg_attr(rustfmt, rustfmt_skip)]


use std::borrow::Cow;
use quick_protobuf::{MessageRead, MessageWrite, BytesReader, Writer, WriterBackend, Result};
use quick_protobuf::sizeofs::*;
use super::super::*;

#[derive(Debug, Default, PartialEq, Clone)]
pub struct Log<'a> {
    pub timestamp: Option<google::protobuf::Timestamp>,
    pub log_level: mcfs::common::LogLevel,
    pub log_string: Option<mcfs::log::LogString<'a>>,
    pub logged_message: Option<mcfs::log::LoggedMessage<'a>>,
}

impl<'a> MessageRead<'a> for Log<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.timestamp = Some(r.read_message::<google::protobuf::Timestamp>(bytes)?),
                Ok(16) => msg.log_level = r.read_enum(bytes)?,
                Ok(26) => msg.log_string = Some(r.read_message::<mcfs::log::LogString>(bytes)?),
                Ok(34) => msg.logged_message = Some(r.read_message::<mcfs::log::LoggedMessage>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for Log<'a> {
    fn get_size(&self) -> usize {
        0
        + self.timestamp.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.log_level == mcfs::common::LogLevel::LOG_LEVEL_DATA { 0 } else { 1 + sizeof_varint(*(&self.log_level) as u64) }
        + self.log_string.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + self.logged_message.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.timestamp { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.log_level != mcfs::common::LogLevel::LOG_LEVEL_DATA { w.write_with_tag(16, |w| w.write_enum(*&self.log_level as i32))?; }
        if let Some(ref s) = self.log_string { w.write_with_tag(26, |w| w.write_message(s))?; }
        if let Some(ref s) = self.logged_message { w.write_with_tag(34, |w| w.write_message(s))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct LoggedMessage<'a> {
    pub logged_message: mcfs::log::mod_LoggedMessage::OneOflogged_message<'a>,
}

impl<'a> MessageRead<'a> for LoggedMessage<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.logged_message = mcfs::log::mod_LoggedMessage::OneOflogged_message::command(r.read_message::<mcfs::command::Command>(bytes)?),
                Ok(18) => msg.logged_message = mcfs::log::mod_LoggedMessage::OneOflogged_message::data(r.read_message::<mcfs::data::Data>(bytes)?),
                Ok(26) => msg.logged_message = mcfs::log::mod_LoggedMessage::OneOflogged_message::status(r.read_message::<mcfs::status::Status>(bytes)?),
                Ok(34) => msg.logged_message = mcfs::log::mod_LoggedMessage::OneOflogged_message::core_message(r.read_message::<mcfs::core::Message>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for LoggedMessage<'a> {
    fn get_size(&self) -> usize {
        0
        + match self.logged_message {
            mcfs::log::mod_LoggedMessage::OneOflogged_message::command(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::log::mod_LoggedMessage::OneOflogged_message::data(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::log::mod_LoggedMessage::OneOflogged_message::status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::log::mod_LoggedMessage::OneOflogged_message::core_message(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::log::mod_LoggedMessage::OneOflogged_message::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        match self.logged_message {            mcfs::log::mod_LoggedMessage::OneOflogged_message::command(ref m) => { w.write_with_tag(10, |w| w.write_message(m))? },
            mcfs::log::mod_LoggedMessage::OneOflogged_message::data(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mcfs::log::mod_LoggedMessage::OneOflogged_message::status(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mcfs::log::mod_LoggedMessage::OneOflogged_message::core_message(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::log::mod_LoggedMessage::OneOflogged_message::None => {},
    }        Ok(())
    }
}

pub mod mod_LoggedMessage {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOflogged_message<'a> {
    command(mcfs::command::Command),
    data(mcfs::data::Data<'a>),
    status(mcfs::status::Status<'a>),
    core_message(mcfs::core::Message<'a>),
    None,
}

impl<'a> Default for OneOflogged_message<'a> {
    fn default() -> Self {
        OneOflogged_message::None
    }
}

}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct LogString<'a> {
    pub string: Cow<'a, str>,
}

impl<'a> MessageRead<'a> for LogString<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.string = r.read_string(bytes).map(Cow::Borrowed)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for LogString<'a> {
    fn get_size(&self) -> usize {
        0
        + if self.string == "" { 0 } else { 1 + sizeof_len((&self.string).len()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.string != "" { w.write_with_tag(10, |w| w.write_string(&**&self.string))?; }
        Ok(())
    }
}

