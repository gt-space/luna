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
pub enum DeviceStatusCode {
    DEVICE_CONNECTED = 0,
    DEVICE_DISCONNECTED = 1,
    DEVICE_ERROR = 2,
}

impl Default for DeviceStatusCode {
    fn default() -> Self {
        DeviceStatusCode::DEVICE_CONNECTED
    }
}

impl From<i32> for DeviceStatusCode {
    fn from(i: i32) -> Self {
        match i {
            0 => DeviceStatusCode::DEVICE_CONNECTED,
            1 => DeviceStatusCode::DEVICE_DISCONNECTED,
            2 => DeviceStatusCode::DEVICE_ERROR,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for DeviceStatusCode {
    fn from(s: &'a str) -> Self {
        match s {
            "DEVICE_CONNECTED" => DeviceStatusCode::DEVICE_CONNECTED,
            "DEVICE_DISCONNECTED" => DeviceStatusCode::DEVICE_DISCONNECTED,
            "DEVICE_ERROR" => DeviceStatusCode::DEVICE_ERROR,
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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum NodeStatusCode {
    NODE_CONNECTED = 0,
    NODE_DISCONNECTED = 1,
    NODE_ERROR = 2,
}

impl Default for NodeStatusCode {
    fn default() -> Self {
        NodeStatusCode::NODE_CONNECTED
    }
}

impl From<i32> for NodeStatusCode {
    fn from(i: i32) -> Self {
        match i {
            0 => NodeStatusCode::NODE_CONNECTED,
            1 => NodeStatusCode::NODE_DISCONNECTED,
            2 => NodeStatusCode::NODE_ERROR,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for NodeStatusCode {
    fn from(s: &'a str) -> Self {
        match s {
            "NODE_CONNECTED" => NodeStatusCode::NODE_CONNECTED,
            "NODE_DISCONNECTED" => NodeStatusCode::NODE_DISCONNECTED,
            "NODE_ERROR" => NodeStatusCode::NODE_ERROR,
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
                Ok(18) => msg.status = mcfs::status::mod_Status::OneOfstatus::device_status(r.read_message::<mcfs::status::DeviceStatus>(bytes)?),
                Ok(26) => msg.status = mcfs::status::mod_Status::OneOfstatus::channel_status(r.read_message::<mcfs::status::ChannelStatus>(bytes)?),
                Ok(34) => msg.status = mcfs::status::mod_Status::OneOfstatus::node_status(r.read_message::<mcfs::status::NodeStatus>(bytes)?),
                Ok(42) => msg.status = mcfs::status::mod_Status::OneOfstatus::device_info(r.read_message::<mcfs::status::DeviceInfo>(bytes)?),
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
            mcfs::status::mod_Status::OneOfstatus::device_status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::channel_status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::node_status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::device_info(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::status::mod_Status::OneOfstatus::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.status_message != "" { w.write_with_tag(10, |w| w.write_string(&**&self.status_message))?; }
        match self.status {            mcfs::status::mod_Status::OneOfstatus::device_status(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::channel_status(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::node_status(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::device_info(ref m) => { w.write_with_tag(42, |w| w.write_message(m))? },
            mcfs::status::mod_Status::OneOfstatus::None => {},
    }        Ok(())
    }
}

pub mod mod_Status {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfstatus {
    device_status(mcfs::status::DeviceStatus),
    channel_status(mcfs::status::ChannelStatus),
    node_status(mcfs::status::NodeStatus),
    device_info(mcfs::status::DeviceInfo),
    None,
}

impl Default for OneOfstatus {
    fn default() -> Self {
        OneOfstatus::None
    }
}

}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct DeviceStatus {
    pub status: mcfs::status::DeviceStatusCode,
}

impl<'a> MessageRead<'a> for DeviceStatus {
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

impl MessageWrite for DeviceStatus {
    fn get_size(&self) -> usize {
        0
        + if self.status == mcfs::status::DeviceStatusCode::DEVICE_CONNECTED { 0 } else { 1 + sizeof_varint(*(&self.status) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.status != mcfs::status::DeviceStatusCode::DEVICE_CONNECTED { w.write_with_tag(8, |w| w.write_enum(*&self.status as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct ChannelStatus {
    pub channel: mcfs::device::Channel,
    pub status: mcfs::status::ChannelStatusCode,
}

impl<'a> MessageRead<'a> for ChannelStatus {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.channel = r.read_enum(bytes)?,
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
        + if self.channel == mcfs::device::Channel::GPIO { 0 } else { 1 + sizeof_varint(*(&self.channel) as u64) }
        + if self.status == mcfs::status::ChannelStatusCode::CHANNEL_CONNECTED { 0 } else { 1 + sizeof_varint(*(&self.status) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.channel != mcfs::device::Channel::GPIO { w.write_with_tag(8, |w| w.write_enum(*&self.channel as i32))?; }
        if self.status != mcfs::status::ChannelStatusCode::CHANNEL_CONNECTED { w.write_with_tag(16, |w| w.write_enum(*&self.status as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct NodeStatus {
    pub node: Option<mcfs::device::NodeIdentifier>,
    pub status: mcfs::status::NodeStatusCode,
}

impl<'a> MessageRead<'a> for NodeStatus {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.node = Some(r.read_message::<mcfs::device::NodeIdentifier>(bytes)?),
                Ok(16) => msg.status = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for NodeStatus {
    fn get_size(&self) -> usize {
        0
        + self.node.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.status == mcfs::status::NodeStatusCode::NODE_CONNECTED { 0 } else { 1 + sizeof_varint(*(&self.status) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.node { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.status != mcfs::status::NodeStatusCode::NODE_CONNECTED { w.write_with_tag(16, |w| w.write_enum(*&self.status as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct DeviceInfo {
    pub board_id: u32,
    pub device_type: mcfs::device::DeviceType,
}

impl<'a> MessageRead<'a> for DeviceInfo {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.board_id = r.read_uint32(bytes)?,
                Ok(16) => msg.device_type = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for DeviceInfo {
    fn get_size(&self) -> usize {
        0
        + if self.board_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.board_id) as u64) }
        + if self.device_type == mcfs::device::DeviceType::SERVER { 0 } else { 1 + sizeof_varint(*(&self.device_type) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.board_id != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.board_id))?; }
        if self.device_type != mcfs::device::DeviceType::SERVER { w.write_with_tag(16, |w| w.write_enum(*&self.device_type as i32))?; }
        Ok(())
    }
}

