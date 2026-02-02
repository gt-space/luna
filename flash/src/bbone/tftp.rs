use log::{error, info, warn};
use num_enum::TryFromPrimitive;
use std::{cmp::min, collections::HashMap, ffi::CStr, fmt::{self, Display, Formatter}, io, net::{SocketAddr, UdpSocket}, num::ParseIntError, thread};

pub struct Server {
  files: HashMap<String, Box<[u8]>>,
  socket: UdpSocket,
}

impl Server {
  pub fn new(files: HashMap<String, Box<[u8]>>) -> io::Result<Self> {
    let socket = UdpSocket::bind("0.0.0.0:67")?;
    socket.set_broadcast(true)?;

    Ok(Self { socket, files })
  }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, TryFromPrimitive)]
#[repr(u16)]
enum Opcode {
  ReadRequest = 1,
  WriteRequest = 2,
  Data = 3,
  Ack = 4,
  Error = 5,
  OptionAck = 6,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, TryFromPrimitive)]
#[repr(u16)]
enum ErrorCode {
  NotDefined = 0,
  FileNotFound = 1,
  AccessViolation = 2,
  DiskFull = 3,
  IllegalOperation = 4,
  UnknownTransferId = 5,
  FileAlreadyExists = 6,
  NoSuchUser = 7,
  OptionNegotiationFailed = 8,
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum Packet {
  ReadRequest {
    filename: String,
    mode: String,
    options: TransferOptions,
  },

  WriteRequest {
    filename: String,
    mode: String,
    options: TransferOptions,
  },

  Data {
    block: u16,
    data: Box<[u8]>,
  },

  Ack {
    block: u16,
  },

  Error {
    code: ErrorCode,
    message: String,
  },

  OptionAck {
    options: TransferOptions,
  },
}

impl Display for Packet {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match self {
      Self::ReadRequest { filename, mode, options } => {
        write!(f, "RRQ {filename} ({mode}) {options}")
      },
      Self::WriteRequest { filename, mode, options } => {
        write!(f, "WRQ {filename} ({mode}) {options}")
      },
      Self::Data { block, data } => {
        write!(f, "DATA block {block} ({} bytes)", data.len())
      },
      Self::Ack { block } => {
        write!(f, "ACK block {block}")
      },
      Self::Error { code, message } => {
        write!(f, "ERROR {code:?} {message}")
      },
      Self::OptionAck { options } => {
        write!(f, "OACK {options}")
      },
    }
  }
}

fn eat_string(buffer: &mut &[u8]) -> Result<String> {
  let string = CStr::from_bytes_until_nul(*buffer)
    .map_err(|_| Error::StringUnterminated((*buffer).into()))?
    .to_string_lossy()
    .into_owned();

  *buffer = &buffer[(string.len() + 1)..];
  Ok(string)
}

fn receive(socket: &UdpSocket) -> Result<(Packet, SocketAddr)> {
  let mut buffer = [0; 516];

  let (size, sender) = loop {
    match socket.recv_from(&mut buffer) {
      Ok(ret) => break ret,
      Err(err) => error!("TFTP receive failed: {err}"),
    }
  };

  info!("received tftp packet from {sender}");

  // Clip the buffer to only its received size.
  let mut buffer = &buffer[..size];

  let opcode = u16::from_be_bytes(buffer[..2].try_into().unwrap());
  let opcode = Opcode::try_from_primitive(opcode)
    .map_err(|_| Error::OpcodeUnrecognized(opcode))?;

  buffer = &buffer[2..];

  let packet = match opcode {
    Opcode::ReadRequest | Opcode::WriteRequest => {
      let filename = eat_string(&mut buffer)?;
      let mode = eat_string(&mut buffer)?;
      let options = TransferOptions::try_from(buffer)?;

      match opcode {
        Opcode::ReadRequest => {
          Packet::ReadRequest { filename, mode, options }
        },
        Opcode::WriteRequest => {
          Packet::WriteRequest { filename, mode, options }
        },
        _ => unreachable!(),
      }
    },
    Opcode::Data => {
      let block = u16::from_be_bytes(buffer[..2].try_into().unwrap());
      let data = Box::from(&buffer[2..]);
      Packet::Data { block, data }
    },
    Opcode::Ack => {
      let block = u16::from_be_bytes(buffer[..2].try_into().unwrap());
      Packet::Ack { block }
    },
    Opcode::Error => {
      let code = u16::from_be_bytes(buffer[..2].try_into().unwrap());
      let code = ErrorCode::try_from_primitive(code)
        .map_err(|_| Error::ErrorCodeUnrecognized(code))?;

      buffer = &buffer[2..];
      let message = eat_string(&mut buffer)?;

      Packet::Error { code, message }
    },
    Opcode::OptionAck => {
      let options = TransferOptions::try_from(buffer)?;
      Packet::OptionAck { options }
    },
  };

  Ok((packet, sender))
}

fn send(socket: &UdpSocket, packet: &Packet) -> Result<()> {
  let mut buffer = Vec::new();

  match packet {
    Packet::Data { block, data } => {
      buffer.extend_from_slice(&block.to_be_bytes());
      buffer.extend_from_slice(&data);
    },
    Packet::Ack { block } => {
      buffer.extend_from_slice(&block.to_be_bytes());
    },
    Packet::Error { code, message } => {
      buffer.extend_from_slice(&(*code as u16).to_be_bytes());
      buffer.extend_from_slice(&message.as_bytes());
      buffer.push(0);
    },
    Packet::OptionAck { options } => {
      buffer.extend(options.into_bytes());
    },
    _ => warn!("requested to send client packet: {packet:#?}"),
  }

  socket.send(&buffer)?;
  Ok(())
}

impl Server {
  pub fn serve(&mut self) -> Result<()> {
    loop {
      let (packet, sender) = receive(&self.socket)?;

      // TODO: handle writes
      thread::spawn(|| {
        let Packet::ReadRequest { filename, mode, options } = packet else {
          panic!("non-read request");
        };

        let file = self.files.get(&filename).unwrap();
        let transfer = Transfer::new(file, options);
      });
    }
  }
}

#[derive(Clone, Debug, Default, Eq, Hash, PartialEq)]
struct TransferOptions {
  block_size: Option<usize>,
  timeout: Option<u32>,
  transfer_size: Option<usize>,
  window_size: Option<usize>,
}

impl TryFrom<&[u8]> for TransferOptions {
  type Error = Error;

  fn try_from(mut buffer: &[u8]) -> Result<Self> {
    let mut options = Self::default();

    while !buffer.is_empty() {
      let key = eat_string(&mut buffer)?;
      let value = eat_string(&mut buffer)?;
      options.push(&key, &value)?;
    }

    Ok(options)
  }
}

impl TransferOptions {
  pub fn push(&mut self, key: &str, value: &str) -> Result<()> {
    match key {
      "blksize" => self.block_size = Some(value.parse()?),
      "timeout" => self.timeout = Some(value.parse()?),
      "tsize" => self.transfer_size = Some(value.parse()?),
      "windowsize" => self.window_size = Some(value.parse()?),
      _ => warn!("unknown option {key}"),
    }

    Ok(())
  }

  pub fn into_bytes(&self) -> Vec<u8> {
    let mut bytes = Vec::new();

    let mut push = |key: &[u8], value: String| {
      bytes.extend_from_slice(key);
      bytes.push(0);
      bytes.extend_from_slice(&value.as_bytes());
      bytes.push(0);
    };

    if let Some(block_size) = self.block_size {
      push(b"blksize", block_size.to_string());
    }

    if let Some(timeout) = self.timeout {
      push(b"timeout", timeout.to_string());
    }

    if let Some(transfer_size) = self.transfer_size {
      push(b"tsize", transfer_size.to_string());
    }

    if let Some(window_size) = self.window_size {
      push(b"windowsize", window_size.to_string());
    }

    bytes
  }

  pub fn is_empty(&self) -> bool {
    self.block_size.is_some()
    || self.timeout.is_some()
    || self.transfer_size.is_some()
    || self.window_size.is_some()
  }
}

impl Display for TransferOptions {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    let mut strings = Vec::new();

    if let Some(block_size) = self.block_size {
      strings.push(format!("blksize={block_size}"));
    }

    if let Some(timeout) = self.timeout {
      strings.push(format!("timeout={timeout}"));
    }

    if let Some(transfer_size) = self.transfer_size {
      strings.push(format!("tsize={transfer_size}"));
    }

    if let Some(window_size) = self.window_size {
      strings.push(format!("windowsize={window_size}"));
    }

    let joined = strings.join(" ");
    write!(f, "{joined}")
  }
}

#[derive(Debug)]
struct Transfer<'a> {
  file: &'a [u8],
  options: TransferOptions,
  socket: UdpSocket,
}

impl<'a> Transfer<'a> {
  pub fn new(file: &'a [u8], options: TransferOptions) -> Result<Self> {
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    Ok(Self { file, options, socket })
  }

  pub fn start(&self) -> Result<()> {
    // Send an options acknowledgement only if some options were set.
    if !self.options.is_empty() {
      send(
        &self.socket,
        &Packet::OptionAck { options: self.options.clone() },
      )?;

      if receive(&self.socket)?.0 != (Packet::Ack { block: 0 }) {
        warn!("unexpected non-ack packet");
      }
    }

    let block_size = self.options.block_size.unwrap_or(512);
    let blocks = self.file.len().div_ceil(block_size);
    let window = self.file;

    if blocks > u16::MAX as usize {
      error!("too many blocks");
    }

    for b in 1..=blocks {
      let block_len = min(block_size, window.len());

      let packet = Packet::Data {
        block: b as u16,
        data: Box::from(&window[..block_len]),
      };

      send(&self.socket, &packet)?;

      if receive(&self.socket)?.0 != (Packet::Ack { block: b as u16 }) {
        warn!("bad ack");
      }
    }

    Ok(())
  }
}

#[derive(Debug)]
pub enum Error {
  ErrorCodeUnrecognized(u16),
  IO(io::Error),
  OpcodeUnrecognized(u16),
  OptionMalformed,
  StringUnterminated(Box<[u8]>),
}

impl Display for Error {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match self {
      Self::ErrorCodeUnrecognized(code) => write!(f, "unrecognized error code: {code}"),
      Self::IO(error) => write!(f, "io: {error}"),
      Self::OpcodeUnrecognized(code) => write!(f, "unrecognized opcode: {code}"),
      Self::OptionMalformed => write!(f, "malformed option"),
      Self::StringUnterminated(bytes) => {
        write!(f, "unterminated C-string: {bytes:?}")
      },
    }
  }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
  fn from(error: io::Error) -> Self {
    Self::IO(error)
  }
}

impl From<ParseIntError> for Error {
  fn from(_: ParseIntError) -> Self {
    Self::OptionMalformed
  }
}

pub type Result<T> = std::result::Result<T, Error>;
