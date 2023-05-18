// Automatically generated rust module for 'core.proto' file

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
pub struct Message<'a> {
    pub timestamp: Option<google::protobuf::Timestamp>,
    pub board_id: u32,
    pub content: mcfs::core::mod_Message::OneOfcontent<'a>,
}

impl<'a> MessageRead<'a> for Message<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.timestamp = Some(r.read_message::<google::protobuf::Timestamp>(bytes)?),
                Ok(16) => msg.board_id = r.read_uint32(bytes)?,
                Ok(26) => msg.content = mcfs::core::mod_Message::OneOfcontent::command(r.read_message::<mcfs::command::Command>(bytes)?),
                Ok(34) => msg.content = mcfs::core::mod_Message::OneOfcontent::data(r.read_message::<mcfs::data::Data>(bytes)?),
                Ok(42) => msg.content = mcfs::core::mod_Message::OneOfcontent::status(r.read_message::<mcfs::status::Status>(bytes)?),
                Ok(50) => msg.content = mcfs::core::mod_Message::OneOfcontent::procedure(r.read_message::<mcfs::procedure::Procedure>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for Message<'a> {
    fn get_size(&self) -> usize {
        0
        + self.timestamp.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.board_id == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.board_id) as u64) }
        + match self.content {
            mcfs::core::mod_Message::OneOfcontent::command(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::core::mod_Message::OneOfcontent::data(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::core::mod_Message::OneOfcontent::status(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::core::mod_Message::OneOfcontent::procedure(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::core::mod_Message::OneOfcontent::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.timestamp { w.write_with_tag(10, |w| w.write_message(s))?; }
        if self.board_id != 0u32 { w.write_with_tag(16, |w| w.write_uint32(*&self.board_id))?; }
        match self.content {            mcfs::core::mod_Message::OneOfcontent::command(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mcfs::core::mod_Message::OneOfcontent::data(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::core::mod_Message::OneOfcontent::status(ref m) => { w.write_with_tag(42, |w| w.write_message(m))? },
            mcfs::core::mod_Message::OneOfcontent::procedure(ref m) => { w.write_with_tag(50, |w| w.write_message(m))? },
            mcfs::core::mod_Message::OneOfcontent::None => {},
    }        Ok(())
    }
}

pub mod mod_Message {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfcontent<'a> {
    command(mcfs::command::Command),
    data(mcfs::data::Data<'a>),
    status(mcfs::status::Status<'a>),
    procedure(mcfs::procedure::Procedure<'a>),
    None,
}

impl<'a> Default for OneOfcontent<'a> {
    fn default() -> Self {
        OneOfcontent::None
    }
}

}

