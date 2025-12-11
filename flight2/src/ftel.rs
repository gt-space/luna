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
      return Err(FtelSocketError::Resolution);
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
    let mut total_packets = (state_bytes.len() / FTEL_PACKET_PAYLOAD_LENGTH + 1) as u8;
    if state_bytes.len() % FTEL_PACKET_PAYLOAD_LENGTH != 0 {
      total_packets += 1;
    }

    buf[0] = self.messages_sent as u8;
    buf[2] = total_packets;
    &buf[3..=4].copy_from_slice(&u16::to_be_bytes(state_bytes.len() as u16));
    &xor_buf[0..FTEL_PACKET_METADATA_LENGTH].copy_from_slice(&buf[0..FTEL_PACKET_METADATA_LENGTH]);
    xor_buf[1] = total_packets - 1;

    let mut current_packet = 0;
    let mut remaining = state_bytes.len() as i32;
    while remaining > 0 {
      let payload_length = min(remaining as usize, FTEL_PACKET_PAYLOAD_LENGTH);
      buf[1] = current_packet;

      // Copy over a slice of the to the buffer and send, accumulating the XOR packet in the process.
      // Will panic if the slice lengths don't match. Scary!
      &buf[FTEL_PACKET_METADATA_LENGTH..].copy_from_slice(&state_bytes[current_packet as usize * FTEL_PACKET_PAYLOAD_LENGTH..current_packet as usize * FTEL_PACKET_PAYLOAD_LENGTH + payload_length]);
      for i in 0..remaining as usize {
        xor_buf[i + FTEL_PACKET_METADATA_LENGTH] ^= buf[i + FTEL_PACKET_METADATA_LENGTH];
      }

      let packet_length = remaining as usize + FTEL_PACKET_METADATA_LENGTH;
      let bytes_written = self.socket.send_to(&buf[..packet_length], &dest_addr)?;
      if bytes_written != packet_length {
        return Err(FtelSocketError::SocketWrite(bytes_written, packet_length));
      }

      current_packet += 1;
      remaining -= FTEL_PACKET_PAYLOAD_LENGTH as i32;
    }

    // Send the XOR Packet
    let bytes_written = self.socket.send_to(&xor_buf, &dest_addr)?;
    if bytes_written != xor_buf.len() {
      return Err(FtelSocketError::SocketWrite(bytes_written, xor_buf.len()));
    }
    
    Ok(())
  }
}

type Result<T> = ::std::result::Result<T, FtelSocketError>;

pub enum FtelSocketError {
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

impl From<io::Error> for FtelSocketError {
  fn from(error: io::Error) -> Self {
    FtelSocketError::Transport(error)
  }
}

impl From<postcard::Error> for FtelSocketError {
  fn from(error: postcard::Error) -> Self {
    FtelSocketError::Postcard(error)
  }
}

impl fmt::Display for FtelSocketError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Resolution => write!(f, "Couldn't resolve binding socket address."),
      Self::SocketWrite(written, size) => write!(f, "Wrote {written} bytes of a message of size {size} bytes."),
      Self::Postcard(e) => write!(f, "Postcard error: {e}"),
      Self::Transport(e) => write!(f, "Transport error: {e}")
    }
  }
}