use log::{debug, error, info, warn};
use num_enum::TryFromPrimitive;
use std::{fmt::{self, Display, Formatter}, io, net::{Ipv4Addr, SocketAddr, UdpSocket}};

// BOOTP fixed packet length (RFC 951):
//   op(1) + htype(1) + hlen(1) + hops(1) + xid(4) + secs(2) + flags(2)
//   + ciaddr(4) + yiaddr(4) + siaddr(4) + giaddr(4) + chaddr(16)
//   + sname(64) + file(128) + vend(64) = 300
const PACKET_LEN: usize = 300;

// DHCP magic cookie (RFC 2131) at the start of the vendor area.
const DHCP_MAGIC: [u8; 4] = [99, 130, 83, 99];

pub struct Server {
  socket: UdpSocket,
  server_ip: Ipv4Addr,
  client_ip: Ipv4Addr,
}

impl Server {
  pub fn new(server_ip: Ipv4Addr, client_ip: Ipv4Addr) -> io::Result<Self> {
    let socket = UdpSocket::bind("0.0.0.0:67")?;
    socket.set_broadcast(true)?;
    Ok(Self { socket, server_ip, client_ip })
  }

  /// Block until a BOOTP/DHCP request arrives, negotiate the handshake,
  /// and reply with the given filename, client IP, and server IP.
  ///
  /// Plain BOOTP (used by the AM335x ROM) is a single request → reply.
  /// DHCP (used by U-Boot's SPL) is a four-way handshake:
  ///   DISCOVER → OFFER → REQUEST → ACK.
  pub fn respond(&self, filename: &str) -> Result<()> {
    let request = loop {
      let (packet, _) = receive(&self.socket)?;
      if packet.op == Op::Request {
        info!("{packet}");
        break packet;
      }
      warn!("ignoring non-request: {packet}");
    };

    let mut file = [0u8; 128];
    let name = filename.as_bytes();
    file[..name.len()].copy_from_slice(name);

    let mut reply = Packet {
      op: Op::Reply,
      htype: request.htype,
      hlen: request.hlen,
      xid: request.xid,
      flags: request.flags,
      yiaddr: self.client_ip,
      siaddr: self.server_ip,
      chaddr: request.chaddr,
      file,
      vend: [0u8; 64],
    };

    let octets = self.server_ip.octets();
    let broadcast = Ipv4Addr::new(octets[0], octets[1], octets[2], 255);
    let dest = SocketAddr::new(broadcast.into(), 68);

    if request.dhcp_message_type().is_some() {
      // DHCPDISCOVER → DHCPOFFER
      reply.set_dhcp_options(2, self.server_ip);
      self.socket.send_to(&reply.serialize(), dest)?;
      debug!("bootp -> {reply} file={filename}");
      info!("{reply} file={filename}");

      // Wait for DHCPREQUEST with matching transaction ID.
      loop {
        let (packet, _) = receive(&self.socket)?;
        if packet.op == Op::Request
          && packet.xid == request.xid
          && packet.dhcp_message_type() == Some(3)
        {
          info!("{packet}");
          break;
        }
      }

      // DHCPREQUEST → DHCPACK
      reply.set_dhcp_options(5, self.server_ip);
      self.socket.send_to(&reply.serialize(), dest)?;
      debug!("bootp -> {reply} file={filename}");
      info!("{reply} file={filename}");
    } else {
      // Plain BOOTP: single request → reply.
      self.socket.send_to(&reply.serialize(), dest)?;
      debug!("bootp -> {reply} yiaddr={} siaddr={} file={filename}", reply.yiaddr, reply.siaddr);
      info!("{reply} file={filename}");
    }

    Ok(())
  }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
enum Op {
  Request = 1,
  Reply = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Packet {
  op: Op,
  htype: u8,
  hlen: u8,
  xid: u32,
  flags: u16,
  yiaddr: Ipv4Addr,
  siaddr: Ipv4Addr,
  chaddr: [u8; 16],
  file: [u8; 128],
  vend: [u8; 64],
}

impl Packet {
  fn parse(bytes: &[u8]) -> Result<Self> {
    if bytes.len() < PACKET_LEN {
      return Err(Error::PacketTooShort(bytes.len()));
    }

    Ok(Self {
      op: Op::try_from_primitive(bytes[0])
        .map_err(|_| Error::OpUnrecognized(bytes[0]))?,
      htype: bytes[1],
      hlen: bytes[2],
      xid: u32::from_be_bytes(bytes[4..8].try_into().unwrap()),
      flags: u16::from_be_bytes(bytes[10..12].try_into().unwrap()),
      yiaddr: Ipv4Addr::from(<[u8; 4]>::try_from(&bytes[16..20]).unwrap()),
      siaddr: Ipv4Addr::from(<[u8; 4]>::try_from(&bytes[20..24]).unwrap()),
      chaddr: bytes[28..44].try_into().unwrap(),
      file: bytes[108..236].try_into().unwrap(),
      vend: bytes[236..300].try_into().unwrap(),
    })
  }

  fn serialize(&self) -> [u8; PACKET_LEN] {
    let mut bytes = [0u8; PACKET_LEN];
    bytes[0] = self.op as u8;
    bytes[1] = self.htype;
    bytes[2] = self.hlen;
    bytes[4..8].copy_from_slice(&self.xid.to_be_bytes());
    bytes[10..12].copy_from_slice(&self.flags.to_be_bytes());
    bytes[16..20].copy_from_slice(&self.yiaddr.octets());
    bytes[20..24].copy_from_slice(&self.siaddr.octets());
    bytes[28..44].copy_from_slice(&self.chaddr);
    bytes[108..236].copy_from_slice(&self.file);
    bytes[236..300].copy_from_slice(&self.vend);
    bytes
  }

  /// Extract the DHCP message type (option 53) from the vendor area,
  /// or `None` if this is a plain BOOTP packet.
  fn dhcp_message_type(&self) -> Option<u8> {
    if self.vend[0..4] != DHCP_MAGIC {
      return None;
    }
    let mut i = 4;
    while i < self.vend.len() {
      match self.vend[i] {
        255 => break,             // End
        0 => i += 1,              // Padding
        53 if i + 2 < self.vend.len() && self.vend[i + 1] == 1 => {
          return Some(self.vend[i + 2]);
        }
        _ => {
          if i + 1 >= self.vend.len() { break; }
          i += 2 + self.vend[i + 1] as usize;
          continue;
        }
      }
    }
    None
  }

  /// Write DHCP options into the vendor area.
  fn set_dhcp_options(&mut self, msg_type: u8, server_ip: Ipv4Addr) {
    self.vend = [0u8; 64];
    let ip = server_ip.octets();
    let mut i = 0;

    // Magic cookie
    self.vend[i..i + 4].copy_from_slice(&DHCP_MAGIC);
    i += 4;

    // Option 53: DHCP Message Type
    self.vend[i..i + 3].copy_from_slice(&[53, 1, msg_type]);
    i += 3;

    // Option 54: Server Identifier
    self.vend[i..i + 2].copy_from_slice(&[54, 4]);
    self.vend[i + 2..i + 6].copy_from_slice(&ip);
    i += 6;

    // Option 1: Subnet Mask
    self.vend[i..i + 2].copy_from_slice(&[1, 4]);
    self.vend[i + 2..i + 6].copy_from_slice(&[255, 255, 255, 0]);
    i += 6;

    // Option 51: Lease Time (max)
    self.vend[i..i + 2].copy_from_slice(&[51, 4]);
    self.vend[i + 2..i + 6].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);
    i += 6;

    // Option 255: End
    self.vend[i] = 255;
  }

  fn mac(&self) -> String {
    let len = (self.hlen as usize).min(self.chaddr.len());
    self.chaddr[..len]
      .iter()
      .map(|b| format!("{b:02x}"))
      .collect::<Vec<_>>()
      .join(":")
  }
}

impl Display for Packet {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match (self.op, self.dhcp_message_type()) {
      (Op::Request, Some(1)) => write!(f, "DHCPDISCOVER"),
      (Op::Request, Some(3)) => write!(f, "DHCPREQUEST"),
      (Op::Reply, Some(2)) => write!(f, "DHCPOFFER"),
      (Op::Reply, Some(5)) => write!(f, "DHCPACK"),
      (Op::Request, _) => write!(f, "BOOTREQUEST"),
      (Op::Reply, _) => write!(f, "BOOTREPLY"),
    }?;
    write!(f, " xid={:#010x} mac={}", self.xid, self.mac())
  }
}

fn receive(socket: &UdpSocket) -> Result<(Packet, SocketAddr)> {
  // 576 bytes: minimum IP reassembly buffer, accommodates DHCP-extended
  // BOOTP packets. We only parse the first 300 bytes.
  let mut buffer = [0u8; 576];

  let (size, sender) = loop {
    match socket.recv_from(&mut buffer) {
      Ok(ret) => break ret,
      Err(err) => error!("BOOTP receive failed: {err}"),
    }
  };

  let packet = Packet::parse(&buffer[..size])?;
  debug!("bootp <- {sender}: {packet} flags={:#06x}", packet.flags);
  Ok((packet, sender))
}

#[derive(Debug)]
pub enum Error {
  IO(io::Error),
  OpUnrecognized(u8),
  PacketTooShort(usize),
}

impl Display for Error {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match self {
      Self::IO(error) => write!(f, "io: {error}"),
      Self::OpUnrecognized(op) => write!(f, "unrecognized op: {op}"),
      Self::PacketTooShort(len) => write!(f, "packet too short: {len} bytes"),
    }
  }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
  fn from(error: io::Error) -> Self {
    Self::IO(error)
  }
}

pub type Result<T> = std::result::Result<T, Error>;
