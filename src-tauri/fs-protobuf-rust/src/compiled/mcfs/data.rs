// Automatically generated rust module for 'data.proto' file

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
pub struct Data<'a> {
    pub node_data: Vec<mcfs::data::NodeData<'a>>,
}

impl<'a> MessageRead<'a> for Data<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.node_data.push(r.read_message::<mcfs::data::NodeData>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for Data<'a> {
    fn get_size(&self) -> usize {
        0
        + self.node_data.iter().map(|s| 1 + sizeof_len((s).get_size())).sum::<usize>()
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        for s in &self.node_data { w.write_with_tag(10, |w| w.write_message(s))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BoolArray {
    pub data: Vec<bool>,
}

impl<'a> MessageRead<'a> for BoolArray {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed(bytes, |r, bytes| Ok(r.read_bool(bytes)?))?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BoolArray {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.iter().map(|s| sizeof_varint(*(s) as u64)).sum::<usize>()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_with_tag(10, &self.data, |w, m| w.write_bool(*m), &|m| sizeof_varint(*(m) as u64))?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct I32Array {
    pub data: Vec<i32>,
}

impl<'a> MessageRead<'a> for I32Array {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed(bytes, |r, bytes| Ok(r.read_sint32(bytes)?))?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for I32Array {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.iter().map(|s| sizeof_sint32(*(s))).sum::<usize>()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_with_tag(10, &self.data, |w, m| w.write_sint32(*m), &|m| sizeof_sint32(*(m)))?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct U32Array {
    pub data: Vec<u32>,
}

impl<'a> MessageRead<'a> for U32Array {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed(bytes, |r, bytes| Ok(r.read_uint32(bytes)?))?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for U32Array {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.iter().map(|s| sizeof_varint(*(s) as u64)).sum::<usize>()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_with_tag(10, &self.data, |w, m| w.write_uint32(*m), &|m| sizeof_varint(*(m) as u64))?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct I64Array {
    pub data: Vec<i32>,
}

impl<'a> MessageRead<'a> for I64Array {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed(bytes, |r, bytes| Ok(r.read_int32(bytes)?))?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for I64Array {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.iter().map(|s| sizeof_varint(*(s) as u64)).sum::<usize>()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_with_tag(10, &self.data, |w, m| w.write_int32(*m), &|m| sizeof_varint(*(m) as u64))?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct U64Array {
    pub data: Vec<u32>,
}

impl<'a> MessageRead<'a> for U64Array {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed(bytes, |r, bytes| Ok(r.read_uint32(bytes)?))?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for U64Array {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.iter().map(|s| sizeof_varint(*(s) as u64)).sum::<usize>()) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_with_tag(10, &self.data, |w, m| w.write_uint32(*m), &|m| sizeof_varint(*(m) as u64))?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct F32Array<'a> {
    pub data: Cow<'a, [f32]>,
}

impl<'a> MessageRead<'a> for F32Array<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed_fixed(bytes)?.into(),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for F32Array<'a> {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.len() * 4) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_fixed_with_tag(10, &self.data)?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct F64Array<'a> {
    pub data: Cow<'a, [f64]>,
}

impl<'a> MessageRead<'a> for F64Array<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.data = r.read_packed_fixed(bytes)?.into(),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for F64Array<'a> {
    fn get_size(&self) -> usize {
        0
        + if self.data.is_empty() { 0 } else { 1 + sizeof_len(self.data.len() * 8) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        w.write_packed_fixed_with_tag(10, &self.data)?;
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct NodeData<'a> {
    pub node: Option<mcfs::device::NodeIdentifier>,
    pub timestamp: Option<google::protobuf::Timestamp>,
    pub micros_offsets: Vec<u32>,
    pub data_points: mcfs::data::mod_NodeData::OneOfdata_points<'a>,
}

impl<'a> MessageRead<'a> for NodeData<'a> {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.node = Some(r.read_message::<mcfs::device::NodeIdentifier>(bytes)?),
                Ok(18) => msg.timestamp = Some(r.read_message::<google::protobuf::Timestamp>(bytes)?),
                Ok(26) => msg.micros_offsets = r.read_packed(bytes, |r, bytes| Ok(r.read_uint32(bytes)?))?,
                Ok(34) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::bool_array(r.read_message::<mcfs::data::BoolArray>(bytes)?),
                Ok(42) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::i32_array(r.read_message::<mcfs::data::I32Array>(bytes)?),
                Ok(50) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::u32_array(r.read_message::<mcfs::data::U32Array>(bytes)?),
                Ok(58) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::i64_array(r.read_message::<mcfs::data::I64Array>(bytes)?),
                Ok(66) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::u64_array(r.read_message::<mcfs::data::U64Array>(bytes)?),
                Ok(74) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::f32_array(r.read_message::<mcfs::data::F32Array>(bytes)?),
                Ok(82) => msg.data_points = mcfs::data::mod_NodeData::OneOfdata_points::f64_array(r.read_message::<mcfs::data::F64Array>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl<'a> MessageWrite for NodeData<'a> {
    fn get_size(&self) -> usize {
        0
        + self.node.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + self.timestamp.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.micros_offsets.is_empty() { 0 } else { 1 + sizeof_len(self.micros_offsets.iter().map(|s| sizeof_varint(*(s) as u64)).sum::<usize>()) }
        + match self.data_points {
            mcfs::data::mod_NodeData::OneOfdata_points::bool_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::i32_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::u32_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::i64_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::u64_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::f32_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::f64_array(ref m) => 1 + sizeof_len((m).get_size()),
            mcfs::data::mod_NodeData::OneOfdata_points::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.node { w.write_with_tag(10, |w| w.write_message(s))?; }
        if let Some(ref s) = self.timestamp { w.write_with_tag(18, |w| w.write_message(s))?; }
        w.write_packed_with_tag(26, &self.micros_offsets, |w, m| w.write_uint32(*m), &|m| sizeof_varint(*(m) as u64))?;
        match self.data_points {            mcfs::data::mod_NodeData::OneOfdata_points::bool_array(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::i32_array(ref m) => { w.write_with_tag(42, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::u32_array(ref m) => { w.write_with_tag(50, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::i64_array(ref m) => { w.write_with_tag(58, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::u64_array(ref m) => { w.write_with_tag(66, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::f32_array(ref m) => { w.write_with_tag(74, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::f64_array(ref m) => { w.write_with_tag(82, |w| w.write_message(m))? },
            mcfs::data::mod_NodeData::OneOfdata_points::None => {},
    }        Ok(())
    }
}

pub mod mod_NodeData {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfdata_points<'a> {
    bool_array(mcfs::data::BoolArray),
    i32_array(mcfs::data::I32Array),
    u32_array(mcfs::data::U32Array),
    i64_array(mcfs::data::I64Array),
    u64_array(mcfs::data::U64Array),
    f32_array(mcfs::data::F32Array<'a>),
    f64_array(mcfs::data::F64Array<'a>),
    None,
}

impl<'a> Default for OneOfdata_points<'a> {
    fn default() -> Self {
        OneOfdata_points::None
    }
}

}

