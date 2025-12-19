use socket2::{Domain, Protocol, Socket, Type, SockAddr};
use std::{cmp::min, fmt, io, net::{SocketAddr, ToSocketAddrs}, time::{Duration, Instant}};
use common::comm::flight::{FTEL_DSCP, FTEL_MTU_TRANSMISSON_LENGTH, FTEL_PACKET_METADATA_LENGTH, FTEL_PACKET_PAYLOAD_LENGTH};

pub struct FtelSocket {
  socket: Socket,
  last_sent: Option<Instant>,
  update_rate: Duration,
  messages_sent: u32,
}

impl FtelSocket {
  /// Creates a dedicated, one-way IP datagram channel to FTel.
  pub fn init(address: impl ToSocketAddrs, update_rate: Duration)
  -> Result<FtelSocket> {
    let Some(address) = address.to_socket_addrs()?.next() else {
      return Err(Error::Resolution);
    };

    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.bind(&SockAddr::from(address))?;
    socket.set_nonblocking(true)?;
    socket.set_tos(FTEL_DSCP << 2)?;
    Ok(Self {
      socket,
      last_sent: None,
      update_rate,
      messages_sent: 0
    })
  }

  /// Determines if enough time has passed to send the message over Ftel.
  /// If enough time has, the message is sent. 
  /// 
  /// `Result::Err(e)` is returned if there was any issue with sending or
  /// serializing the message.
  /// 
  /// `Result::Ok(true)` is returned if the entire message was sent 
  /// successfully, and `Result::Ok(false)` if enough time hasn't elapsed to
  /// send the message.
  pub fn reverse_poll<T: serde::Serialize>(
    &mut self,
    dest_addr: &SocketAddr,
    message: &T
  ) -> Result<bool> {
    if let Some(time_point) = self.last_sent {
      if Instant::now().duration_since(time_point) < self.update_rate {
        return Ok(false);
      }
    }

    // We reset the timer even if the message wasn't fully sent to give the 
    // kernel some time to potentially resolve the issue with the socket.
    
    let res = self.send(dest_addr, message);
    self.messages_sent += 1;
    self.last_sent = Some(Instant::now());
    res.map(|_| true)
  }

  fn accumulate_xor_payload(xor_payload: &mut [u8], message: &[u8]) {
    let mut bytes_xored = 0;
    while bytes_xored < message.len() {
        let length = min(xor_payload.len(), message.len() - bytes_xored);
        for i in 0..length {
            xor_payload[i] ^= message[bytes_xored + i];
        }

        bytes_xored += length;
    }
  }

  fn send<T: serde::Serialize>(
    &mut self, 
    dest_addr: &SocketAddr, 
    message: &T
  ) -> Result<()> {
    let dest_addr = SockAddr::from(*dest_addr);
    // TODO Replace this with a more performat buffer allocation method?
    let state_bytes = postcard::to_allocvec(message)?;
    let mut buf: [u8; FTEL_MTU_TRANSMISSON_LENGTH] = [0; FTEL_MTU_TRANSMISSON_LENGTH];
    let mut xor_buf: [u8; FTEL_MTU_TRANSMISSON_LENGTH] = [0; FTEL_MTU_TRANSMISSON_LENGTH];
    
    // computes the total number of packets which need to be sent for this
    // VehicleState, which includes the XOR packet and any overfill packets for
    // VehicleStates whose length is not divisible by 255.
    let total_packets = state_bytes.len().div_ceil(FTEL_PACKET_PAYLOAD_LENGTH) as u8 + 1;

    buf[0] = self.messages_sent as u8;
    buf[2] = total_packets;
    buf[3..=4].copy_from_slice(&u16::to_be_bytes(state_bytes.len() as u16));
    xor_buf[0..FTEL_PACKET_METADATA_LENGTH].copy_from_slice(&buf[0..FTEL_PACKET_METADATA_LENGTH]);
    xor_buf[1] = total_packets - 1;

    // Compute XOR packet.
    FtelSocket::accumulate_xor_payload(&mut xor_buf[FTEL_PACKET_METADATA_LENGTH..], &state_bytes[..]);

    let mut current_packet = 0;
    let mut remaining = state_bytes.len() as i32;
    while remaining > 0 {
      let payload_length = min(remaining as usize, FTEL_PACKET_PAYLOAD_LENGTH);
      buf[1] = current_packet;

      // Copy over a slice of the message to the buffer.
      // Will panic if the slice lengths don't match. Scary!
      buf[FTEL_PACKET_METADATA_LENGTH..].copy_from_slice(&state_bytes[current_packet as usize * FTEL_PACKET_PAYLOAD_LENGTH..current_packet as usize * FTEL_PACKET_PAYLOAD_LENGTH + payload_length]);

      let packet_length = payload_length as usize + FTEL_PACKET_METADATA_LENGTH;
      let bytes_written = self.socket.send_to(&buf[..packet_length], &dest_addr)?;
      if bytes_written != packet_length {
        return Err(Error::SocketWrite(bytes_written, packet_length));
      }

      current_packet += 1;
      remaining -= FTEL_PACKET_PAYLOAD_LENGTH as i32;
    }

    // Send the XOR Packet
    let bytes_written = self.socket.send_to(&xor_buf, &dest_addr)?;
    if bytes_written != xor_buf.len() {
      return Err(Error::SocketWrite(bytes_written, xor_buf.len()));
    }
    
    Ok(())
  }
}

type Result<T> = ::std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
  /// There was an issue trying to resolve the socket address to bind to.
  Resolution,
  /// There was an issue with the underlying UDP socket.
  Transport(io::Error),
  /// There was an issue serializing the message.
  Postcard(postcard::Error),
  /// The number of bytes written to the socket did not match the size of the 
  /// serialized message.
  /// Field 0 is the number of bytes written, field 1 is the size of the
  /// serialized message. 
  SocketWrite(usize, usize),
}

impl From<io::Error> for Error {
  fn from(error: io::Error) -> Self {
    Error::Transport(error)
  }
}

impl From<postcard::Error> for Error {
  fn from(error: postcard::Error) -> Self {
    Error::Postcard(error)
  }
}

impl fmt::Display for Error {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Resolution => write!(f, "Couldn't resolve binding socket address."),
      Self::SocketWrite(written, size) => write!(f, "Wrote {written} bytes of a message of size {size} bytes."),
      Self::Postcard(e) => write!(f, "Postcard error: {e}"),
      Self::Transport(e) => write!(f, "Transport error: {e}")
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::{mem::MaybeUninit, sync::atomic::{AtomicI32, Ordering}};
  static IDENTIFIER: AtomicI32 = AtomicI32::new(4573);
  
  fn initalize(duration: Duration) -> (FtelSocket, SocketAddr, Socket) {
    let identifier = IDENTIFIER.fetch_add(2, Ordering::Relaxed);

    let address = format!("127.0.0.1:{}", identifier + 1).to_socket_addrs().unwrap().next().unwrap();
    let ftel = FtelSocket::init(address, duration).unwrap();

    let mocket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).unwrap();
    let address = format!("127.0.0.1:{identifier}").to_socket_addrs().unwrap().next().unwrap();
    mocket.bind(&SockAddr::from(address)).unwrap();
    mocket.set_nonblocking(true).unwrap();

    (ftel, address, mocket)
  }

  #[test]
  fn empty_message() {
    let (mut ftel, dest, mocket) = initalize(Duration::ZERO);

    ftel.reverse_poll(&dest, &()).unwrap();
    let mut buf = [MaybeUninit::uninit(); 32];
    let read = loop {
      match mocket.recv_from(&mut buf) {
        Ok((r, _)) => break r,
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => continue,
        Err(e) => panic!("{e}"),
      }
    };
    let serialized: Vec<u8> = buf[..read].iter().map(|b| unsafe { b.assume_init() }).collect();
    
    assert_eq!(serialized[0] as u32, ftel.messages_sent - 1);
    assert_eq!(serialized[1], 0);
    assert_eq!(serialized[2], 1);
    panic!("Payload length: {}", u16::from_be_bytes(serialized[3..5].try_into().unwrap()));
  }
}