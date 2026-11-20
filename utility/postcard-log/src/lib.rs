use std::{
    collections::HashSet,
    io::{self, BufRead, Seek, SeekFrom},
};

use common::comm::{fc_sensors::Imu, VehicleState};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use serde_json::Value;

const MAX_FRAME_SIZE: u64 = 100_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum LogKind {
    VehicleState,
    Imu,
}

// The wire format is f64 Unix seconds followed by the logger's state struct.
#[derive(Deserialize)]
struct Timestamped<T> {
    timestamp: f64,
    state: T,
}

pub struct Record {
    pub timestamp: f64,
    pub offset: u64,
    pub state: Value,
}

pub struct FrameReader<R> {
    inner: R,
    payload: Vec<u8>,
    offset: u64,
    end: u64,
    kind: Option<LogKind>,
    require_full_payload: bool,
}

impl<R: BufRead> FrameReader<R> {
    pub fn new(inner: R, end: u64, kind: Option<LogKind>) -> Self {
        Self {
            inner,
            payload: Vec::new(),
            offset: 0,
            end,
            kind,
            require_full_payload: true,
        }
    }

    pub fn with_legacy_decoding(mut self) -> Self {
        self.require_full_payload = false;
        self
    }

    pub fn kind(&self) -> Option<LogKind> {
        self.kind
    }

    pub fn position(&self) -> u64 {
        self.offset
    }

    pub fn into_inner(self) -> R {
        self.inner
    }

    pub fn read_record(&mut self) -> io::Result<Option<Record>> {
        let offset = self.offset;
        if offset == self.end {
            return Ok(None);
        }
        if self.end - offset < 8 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "partial length prefix",
            ));
        }
        let mut prefix = [0; 8];
        self.inner.read_exact(&mut prefix)?;
        let length = u64::from_le_bytes(prefix);
        if length > MAX_FRAME_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("invalid entry length of {length} bytes"),
            ));
        }
        if length == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "zero-length record",
            ));
        }
        if length > self.end - offset - 8 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "partial record payload",
            ));
        }
        self.payload.resize(length as usize, 0);
        self.inner.read_exact(&mut self.payload)?;
        let (kind, timestamp, state) = match self.kind {
            Some(LogKind::VehicleState) => self.decode::<VehicleState>(LogKind::VehicleState),
            Some(LogKind::Imu) => self.decode::<Imu>(LogKind::Imu),
            None => self
                .decode::<VehicleState>(LogKind::VehicleState)
                .or_else(|_| self.decode::<Imu>(LogKind::Imu)),
        }?;
        if self.require_full_payload && !timestamp.is_finite() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "record timestamp is not finite",
            ));
        }
        self.kind = Some(kind);
        self.offset += 8 + length;
        Ok(Some(Record {
            timestamp,
            offset,
            state,
        }))
    }

    fn decode<T: DeserializeOwned + Serialize>(
        &self,
        kind: LogKind,
    ) -> io::Result<(LogKind, f64, Value)> {
        let (record, remainder) = postcard::take_from_bytes::<Timestamped<T>>(&self.payload)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        if self.require_full_payload && !remainder.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "unrecognized bytes at end of record",
            ));
        }
        let state = serde_json::to_value(record.state).map_err(io::Error::other)?;
        Ok((kind, record.timestamp, state))
    }
}

impl<R: BufRead + Seek> FrameReader<R> {
    pub fn seek_to(&mut self, offset: u64) -> io::Result<()> {
        if offset > self.end {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "offset is past the end of the log",
            ));
        }
        if offset != self.offset {
            self.inner.seek(SeekFrom::Start(offset))?;
            self.offset = offset;
        }
        Ok(())
    }
}

#[derive(Default)]
pub struct Columns(HashSet<String>);

impl Columns {
    pub fn observe(&mut self, record: &Record) {
        self.observe_value(&record.state, &mut String::new());
    }

    fn observe_value(&mut self, value: &Value, path: &mut String) {
        use std::fmt::Write;
        let parent_length = path.len();
        match value {
            Value::Object(fields) => {
                for (key, child) in fields {
                    if parent_length > 0 {
                        path.push('.');
                    }
                    path.push_str(key);
                    self.observe_value(child, path);
                    path.truncate(parent_length);
                }
            }
            Value::Array(items) if !items.is_empty() => {
                for (index, item) in items.iter().enumerate() {
                    write!(path, "[{index}]").expect("writing to a string is infallible");
                    self.observe_value(item, path);
                    path.truncate(parent_length);
                }
            }
            _ if !path.is_empty() && !self.0.contains(path.as_str()) => {
                self.0.insert(path.clone());
            }
            _ => {}
        }
    }

    pub fn extend(&mut self, columns: &[String]) {
        self.0.extend(columns.iter().cloned());
    }

    pub fn finish(mut self) -> Vec<String> {
        self.0.remove("timestamp");
        let mut columns: Vec<_> = self.0.into_iter().collect();
        columns.sort_unstable();
        columns.insert(0, "timestamp".into());
        columns
    }
}

enum Step<'a> {
    Key(&'a str),
    Index(usize),
}

enum Field<'a> {
    Timestamp,
    State(Vec<Step<'a>>),
    Missing,
}

impl<'a> Field<'a> {
    fn parse(mut path: &'a str) -> Self {
        if path == "timestamp" {
            return Self::Timestamp;
        }
        let mut steps = Vec::new();
        while !path.is_empty() {
            path = path.trim_start_matches('.');
            if let Some(array) = path.strip_prefix('[') {
                let Some((index, rest)) = array.split_once(']') else {
                    return Self::Missing;
                };
                let Ok(index) = index.parse() else {
                    return Self::Missing;
                };
                steps.push(Step::Index(index));
                path = rest;
            } else if !path.is_empty() {
                let end = path.find(['.', '[']).unwrap_or(path.len());
                steps.push(Step::Key(&path[..end]));
                path = &path[end..];
            }
        }
        Self::State(steps)
    }

    fn value(&self, record: &Record) -> String {
        match self {
            Self::Timestamp => record.timestamp.to_string(),
            Self::State(steps) => steps
                .iter()
                .try_fold(&record.state, |value, step| match step {
                    Step::Key(key) => value.get(*key),
                    Step::Index(index) => value.get(*index),
                })
                .map_or_else(String::new, value_to_string),
            Self::Missing => String::new(),
        }
    }
}

pub struct Projection<'a>(Vec<Field<'a>>);

impl<'a> Projection<'a> {
    pub fn new(columns: &'a [String]) -> Self {
        Self(columns.iter().map(|name| Field::parse(name)).collect())
    }

    pub fn row(&self, record: &Record) -> Vec<String> {
        self.0.iter().map(|field| field.value(record)).collect()
    }
}

fn value_to_string(value: &Value) -> String {
    match value {
        Value::Null => String::new(),
        Value::String(value) => value.clone(),
        Value::Number(value) => value
            .as_f64()
            .map_or_else(|| value.to_string(), |number| number.to_string()),
        _ => value.to_string(),
    }
}
