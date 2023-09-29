// Automatically generated rust module for 'status.proto' file

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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BoardStatusCode {
    BOARD_CONNECTED = 0,
    BOARD_DISCONNECTED = 1,
    BOARD_ERROR = 2,
}

impl Default for BoardStatusCode {
    fn default() -> Self {
        BoardStatusCode::BOARD_CONNECTED
    }
}

impl From<i32> for BoardStatusCode {
    fn from(i: i32) -> Self {
        match i {
            0 => BoardStatusCode::BOARD_CONNECTED,
            1 => BoardStatusCode::BOARD_DISCONNECTED,
            2 => BoardStatusCode::BOARD_ERROR,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for BoardStatusCode {
    fn from(s: &'a str) -> Self {
        match s {
            "BOARD_CONNECTED" => BoardStatusCode::BOARD_CONNECTED,
            "BOARD_DISCONNECTED" => BoardStatusCode::BOARD_DISCONNECTED,
            "BOARD_ERROR" => BoardStatusCode::BOARD_ERROR,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ChannelStatusCode {
    CHANNEL_CONNECTED = 0,
    CHANNEL_DISCONNECTED = 1,
    CHANNEL_ERROR = 2,
}

impl Default for ChannelStatusCode {
    fn default() -> Self {
        ChannelStatusCode::CHANNEL_CONNECTED
    }
}

impl From<i32> for ChannelStatusCode {
    fn from(i: i32) -> Self {
        match i {
            0 => ChannelStatusCode::CHANNEL_CONNECTED,
            1 => ChannelStatusCode::CHANNEL_DISCONNECTED,
            2 => ChannelStatusCode::CHANNEL_ERROR,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for ChannelStatusCode {
    fn from(s: &'a str) -> Self {
        match s {
            "CHANNEL_CONNECTED" => ChannelStatusCode::CHANNEL_CONNECTED,
            "CHANNEL_DISCONNECTED" => ChannelStatusCode::CHANNEL_DISCONNECTED,
            "CHANNEL_ERROR" => ChannelStatusCode::CHANNEL_ERROR,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct Status<'a> {
    pub status_message: Cow<'a, str>,
    pub status: mcfs::status::mod_Status::OneOfstatus,
}

impl<'a> MessageRead<'a> for Status<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.status_message = r.read_string(bytes).map(Cow::Borrowed)?,
                Ok(18) => msg.status = mcfs::status::mod_Status::OneOfstatus::board_status(r.read_message::<mcfs::status::BoardStatus>(bytes)?),
                Ok(26) => msg.status = mcfs::status::mod_Status::OneOfstatus::channel_status(r.read_message::<mcfs::status::ChannelStatus>(bytes)?),
                Ok(34) => msg.status = mcfs::status::mod_Status::OneOfstatus::board_info(r.read_message::<mcfs::status::BoardInfo>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for Status<'a> {
    fn get_size(&self) -> usize {
        0
        + if self.status_message == "" { 0 } else { 1 + sizeof_len((&self.status_message).len()) }
        + match self.status {
            mcfs::status::mod_Status::OneOfstatus::board_status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::channel_status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::board_info(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.status_message != "" { w.write_with_tag(10, |w| w.write_string(&**&self.status_message))?; }
        match self.status {            mcfs::status::mod_Status::OneOfstatus::board_status(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::channel_status(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::board_info(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::None => {},
    }        Ok(())
    }
}

pub mod mod_Status {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfstatus {
    board_status(mcfs::status::BoardStatus),
    channel_status(mcfs::status::ChannelStatus),
    board_info(mcfs::status::BoardInfo),
    None,
}

impl Default for OneOfstatus {
    fn default() -> Self {
        OneOfstatus::None
    }
}

}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BoardStatus {
    pub status: mcfs::status::BoardStatusCode,
}

impl<'a> MessageRead<'a> for BoardStatus {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.status = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BoardStatus {
    fn get_size(&self) -> usize {
        0
        + if self.status == mcfs::status::BoardStatusCode::BOARD_CONNECTED { 0 } else { 1 + sizeof_varint(*(&self.status) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.status != mcfs::status::BoardStatusCode::BOARD_CONNECTED { w.write_with_tag(8, |w| w.write_enum(*&self.status as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct ChannelStatus {
    pub channel: Option<mcfs::board::ChannelIdentifier>,
    pub status: mcfs::status::ChannelStatusCode,
}

impl<'a> MessageRead<'a> for ChannelStatus {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.channel = Some(r.read_message::<mcfs::board::ChannelIdentifier>(bytes)?),
                Ok(16) => msg.status = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for ChannelStatus {
    fn get_size(&self) -> usize {
        0
        + self.channel.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.status == mcfs::status::ChannelStatusCode::CHANNEL_CONNECTED { 0 } else { 1 + sizeof_varint(*(&self.status) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.channel { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.status != mcfs::status::ChannelStatusCode::CHANNEL_CONNECTED { w.write_with_tag(16, |w| w.write_enum(*&self.status as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BoardInfo {
    pub board_id: u32,
    pub board_type: mcfs::board::BoardType,
}

impl<'a> MessageRead<'a> for BoardInfo {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.board_id = r.read_uint32(bytes)?,
                Ok(16) => msg.board_type = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BoardInfo {
    fn get_size(&self) -> usize {
        0
        + if self.board_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.board_id) as u64) }
        + if self.board_type == mcfs::board::BoardType::SERVER { 0 } else { 1 + sizeof_varint(*(&self.board_type) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.board_id != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.board_id))?; }
        if self.board_type != mcfs::board::BoardType::SERVER { w.write_with_tag(16, |w| w.write_enum(*&self.board_type as i32))?; }
        Ok(())
    }
}

