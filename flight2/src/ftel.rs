use socket2::{Domain, Protocol, Socket, Type, SockAddr};
use std::{fmt, io, net::{SocketAddr, ToSocketAddrs}, time::{Duration, Instant}};

/// This is the value we use in the DSCP field of the `IP_TOS` byte.
/// This is how we differentiate between a packet sent along umbilical versus
/// FTel, as umbilical packets have DSCP field of zeros, which is the default.
const FTEL_DSCP: u32 = 10;

pub struct FtelSocket {
  socket: Socket,
  last_sent: Option<Instant>,
  update_rate: Duration
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
      update_rate
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
  pub fn poll<T: serde::Serialize>(
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
    self.last_sent = Some(Instant::now());
    self.send(dest_addr, message)?;
    Ok(true)
  }

  fn send<T: serde::Serialize>(
    &mut self, 
    dest_addr: &SocketAddr, 
    message: &T
  ) -> Result<()> {
    let dest_addr = SockAddr::from(*dest_addr);
    // TODO Replace this with a more performat buffer allocation method?
    let bytes = postcard::to_allocvec(message)?;
    let bytes_written = self.socket.send_to(&bytes[..], &dest_addr)?;
    if bytes_written != bytes.len() {
      return Err(FtelSocketError::SocketWrite(bytes_written, bytes.len()));
    }
    // Reset the timer again to account for the time spent serializing and 
    // writing to sockets
    self.last_sent = Some(Instant::now());
    
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