use socket2::{Domain, Protocol, Socket, Type, SockAddr};
use std::{fmt, io, net::{SocketAddr, ToSocketAddrs}, slice::Chunks, time::{Duration, Instant}};
use common::comm::{VehicleState, flight::{FTEL_DSCP, FTEL_MTU_TRANSMISSON_LENGTH, FTEL_PACKET_METADATA_LENGTH, FTEL_PACKET_PAYLOAD_LENGTH, PACKET_ID_INDEX, SIZE_RANGE, STATE_ID_INDEX, TOTAL_INDEX}};

/// FtelSocket is the mechanism used to communicate directly through RF to 
/// Servo. Used after umbilical has been detached.
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
    let Some(address) = address.to_socket_addrs()?.find(|a| a.is_ipv4()) else {
      return Err(Error::Resolution);
    };

    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.bind(&SockAddr::from(address))?;
    socket.set_nonblocking(true)?;
    socket.set_tos((FTEL_DSCP as u32) << 2)?;

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
  pub fn send_if_passed_deadline(
    &mut self,
    dest_addr: &SocketAddr,
    state: &VehicleState
  ) -> Result<bool> {
    let serialized = postcard::to_allocvec(state)?;
    self.reverse_poll(dest_addr, &serialized[..])
  }

  /// Sends the message over the FTel socket if the deadline is passed.
  fn reverse_poll(&mut self, dest_addr: &SocketAddr, message: &[u8])
  -> Result<bool> {
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

  /// Splits the message into multiple packets, and sends them to the 
  /// destination address, followed by sending the XOR packet.
  fn send(&mut self, dest_addr: &SocketAddr, message: &[u8]) -> Result<()> {
    fn accumulate_xor_payload(xor_payload: &mut [u8], message: Chunks<u8>) {
      for chunk in message {
        xor_payload.iter_mut().zip(chunk).for_each(|(dest, src)| *dest ^= src);
      }
    }
    
    let dest_addr = SockAddr::from(*dest_addr);
    let mut buf = [0u8; FTEL_MTU_TRANSMISSON_LENGTH];
    let mut xor_buf = [0u8; FTEL_MTU_TRANSMISSON_LENGTH];
    
    // computes the total number of packets which need to be sent for this
    // VehicleState, which includes the XOR packet and any overfill packets for
    // VehicleStates whose length is not divisible by 255.
    let message_chunks = message.chunks(FTEL_PACKET_PAYLOAD_LENGTH);
    let total_packets = message_chunks.len() + 1;
    
    buf[STATE_ID_INDEX] = self.messages_sent as u8;
    buf[TOTAL_INDEX] = total_packets as u8;
    buf[SIZE_RANGE]
      .copy_from_slice(&u16::to_be_bytes(message.len() as u16));
    xor_buf[0..FTEL_PACKET_METADATA_LENGTH]
      .copy_from_slice(&buf[0..FTEL_PACKET_METADATA_LENGTH]);
    xor_buf[PACKET_ID_INDEX] = total_packets as u8 - 1;

    // Compute XOR packet.
    accumulate_xor_payload(
      &mut xor_buf[FTEL_PACKET_METADATA_LENGTH..],
      message_chunks.clone()
    );

    for (i, chunk) in message_chunks.enumerate() {
      buf[PACKET_ID_INDEX] = i as u8;

      // Copy over a slice of the message to the buffer.
      // Will panic if the slice lengths don't match. Scary!
      let packet_length = chunk.len() + FTEL_PACKET_METADATA_LENGTH;
      buf[FTEL_PACKET_METADATA_LENGTH..packet_length].copy_from_slice(chunk);
      
      let bytes_written = self.socket.send_to(&buf[..packet_length], &dest_addr)?;
      if bytes_written != packet_length {
        return Err(Error::SocketWrite(bytes_written, packet_length));
      }
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
  const TIMEOUT_DURATION: Duration = Duration::from_millis(50);
  
  /// Initializes commonly used variables for tests.
  fn initialize(duration: Duration) -> (FtelSocket, SocketAddr, Socket) {
    let identifier = IDENTIFIER.fetch_add(2, Ordering::Relaxed);

    let address = format!("127.0.0.1:{}", identifier + 1).to_socket_addrs().unwrap().next().unwrap();
    let ftel = FtelSocket::init(address, duration).unwrap();

    let mocket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).unwrap();
    let address = 
      format!("127.0.0.1:{identifier}")
      .to_socket_addrs()
      .unwrap()
      .next().unwrap();
    mocket.bind(&SockAddr::from(address)).unwrap();
    mocket.set_nonblocking(true).unwrap();
    mocket.set_recv_tos(true).unwrap();

    (ftel, address, mocket)
  }

  // TODO: Add check for DSCP.
  /// Receives a packet from the socket with a timeout.
  fn recv_packet_from(socket: &Socket) -> Option<Vec<u8>> {
    let mut buf = [MaybeUninit::uninit(); FTEL_MTU_TRANSMISSON_LENGTH * 2];
    let start = Instant::now();
    let read = loop {
       match socket.recv_from(&mut buf) {
        Ok((r, _)) => break r,
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {},
        Err(e) => panic!("{e}"),
      }

      if Instant::now().duration_since(start) > TIMEOUT_DURATION {
        return None;
      }
    };
    
    let packet = buf[..read].iter().map(|b| unsafe { b.assume_init() })
      .collect::<Vec<u8>>();
    Some(packet)
  }

  /// Checks if the packet has any more packets left.
  fn check_if_empty(socket: &Socket) {
    let start = Instant::now();
    loop {
      let res = socket
        .recv_from(&mut [MaybeUninit::uninit(); 1])
        .unwrap_err()
        .kind();

      assert_eq!(
        res,
        io::ErrorKind::WouldBlock
      );

      if Instant::now().duration_since(start) > TIMEOUT_DURATION {
        return;
      }
    }
  }

  /// Ensures that the metadata and invariants between all the packets within
  /// the passed sequence are valid.
  fn validate_packets(
    ftel: &FtelSocket,
    packets: &[&[u8]],
    message: &[u8],
    expected_packet_count: usize
  ) {
    let mut buf = Vec::<u8>::new();

    let data_packets = &packets[..packets.len() - 1];
    let xor_packet = packets.last().unwrap();
    let mut xor_payload = xor_packet[FTEL_PACKET_METADATA_LENGTH..].to_vec();
    for (i, packet) in data_packets.iter().enumerate() {
      assert_eq!(packet[STATE_ID_INDEX] as u32, ftel.messages_sent - 1);
      assert_eq!(packet[PACKET_ID_INDEX] as usize, i);
      assert_eq!(packet[TOTAL_INDEX] as usize, expected_packet_count);
      assert_eq!(
        u16::from_be_bytes(packet[SIZE_RANGE].try_into().unwrap()) as usize,
        message.len()
      );

      let payload = &packet[FTEL_PACKET_METADATA_LENGTH..];
      if i != data_packets.len() - 1 {
        assert_eq!(packet.len(), FTEL_MTU_TRANSMISSON_LENGTH);
      } else {
        assert!(packet.len() == message.len() % FTEL_PACKET_PAYLOAD_LENGTH + FTEL_PACKET_METADATA_LENGTH || packet.len() == FTEL_MTU_TRANSMISSON_LENGTH);
      }

      xor_payload.iter_mut().zip(payload).for_each(|(dest, src)| *dest ^= src);
      buf.extend_from_slice(payload);
    }

    assert_eq!(message, &buf[..]);

    assert_eq!(xor_packet.len(), FTEL_MTU_TRANSMISSON_LENGTH);
    assert_eq!(xor_packet[STATE_ID_INDEX] as u32, ftel.messages_sent - 1);
    assert_eq!(xor_packet[PACKET_ID_INDEX] as usize, packets.len() - 1);
    assert_eq!(xor_packet[TOTAL_INDEX] as usize, expected_packet_count);
    assert_eq!(
      u16::from_be_bytes(xor_packet[SIZE_RANGE].try_into().unwrap()) as usize,
      message.len()
    );

    assert!(xor_payload.iter().all(|b| *b == 0));
  }

  /// Empties all pending packets from a socket.
  fn get_packets(socket: &Socket) -> Vec<Vec<u8>> {
    let mut res = Vec::new();

    while let Some(packet) = recv_packet_from(socket) {
      res.push(packet);
    }

    res
  }

  /// Initializes a packet split with n bytes.
  fn pack_n_bytes(n: usize, expected_packet_count: usize) {
    assert!(n >= 1);

    let (mut ftel, dest, mocket) = initialize(Duration::ZERO);

    let data: Vec<u8> = (0u8..=u8::MAX).cycle().take(n).collect();
    ftel.reverse_poll(&dest, &data[..]).unwrap();
    let packets = get_packets(&mocket);
    check_if_empty(&mocket);

    let packets = &packets.iter().map(|v| &v[..]).collect::<Vec<_>>();
    validate_packets(&ftel, packets, &data[..], expected_packet_count);
  }

  /// Initializes a packet split with n unfilled packets.
  fn validate_n_unfilled_packets(n: usize) {
    pack_n_bytes(
      FTEL_PACKET_PAYLOAD_LENGTH * (n - 1) + FTEL_PACKET_PAYLOAD_LENGTH / 2,
      n + 1
    );
  }

  /// Initializes a packet split with n filled packets.
  fn validate_n_filled_packets(n: usize) {
    pack_n_bytes(FTEL_PACKET_PAYLOAD_LENGTH * n, n + 1);
  }

  /// Initializes a packet split with n bytes, then drops a packet and tests if
  /// XOR recovery holds.
  fn xor_recovery_validation(
    n: usize,
    expected_packet_count: usize,
    dropped_packet: usize
  ) {    
    let (mut ftel, dest, mocket) = initialize(Duration::ZERO);

    let data: Vec<u8> = 
      (0u8..=u8::MAX)
      .cycle()
      .take(n)
      .collect();
    ftel.reverse_poll(&dest, &data[..]).unwrap();
    let packets = get_packets(&mocket);
    check_if_empty(&mocket);

    let packets = &packets.iter().map(|v| &v[..]).collect::<Vec<_>>();
    validate_packets(&ftel, packets, &data[..], expected_packet_count);

    let mut buf = [0u8; FTEL_PACKET_PAYLOAD_LENGTH];
    for (i, packet) in packets[..packets.len() - 1].iter().enumerate() {
      if i == dropped_packet {
        continue;
      }

      buf
        .iter_mut()
        .zip(&packet[FTEL_PACKET_METADATA_LENGTH..])
        .for_each(|(dest, src)| *dest ^= src);
    }

    buf
      .iter_mut()
      .zip(&packets.last().unwrap()[FTEL_PACKET_METADATA_LENGTH..])
      .for_each(|(dest, src)| *dest ^= src);

    assert_eq!(buf, packets[dropped_packet][FTEL_PACKET_METADATA_LENGTH..]);
  }

  /// Initializes a packet split with XOR recovery testing on n filled packets.
  fn xor_recovery_validation_filled(n: usize) {
    xor_recovery_validation(FTEL_PACKET_PAYLOAD_LENGTH * n, n + 1, n / 2);
  }

  /// Initializes a packet split with XOR recovery testing on n unfilled 
  /// packets.
  fn xor_recovery_validation_unfilled(n: usize) {
    xor_recovery_validation(
      FTEL_PACKET_PAYLOAD_LENGTH * (n - 1) + FTEL_PACKET_PAYLOAD_LENGTH / 2,
      n + 1,
      n / 2
    );
  }

  /// On an empty message, only an empty XOR packet should be sent. Technically,
  /// this shouldn't occur since the library user is only able to access the 
  /// method that calls send_if_passed_deadline(), which only accepts a 
  /// VehicleState as the message. However, it's still important to make sure
  /// the underlying packet splitter logic is sound for all scenarios.
  #[test]
  fn empty_message() {
    let (mut ftel, dest, mocket) = initialize(Duration::ZERO);

    let serialized = postcard::to_allocvec(&()).unwrap();
    ftel.reverse_poll(&dest, &serialized[..]).unwrap();
    let packets = get_packets(&mocket);
    check_if_empty(&mocket);
    
    let packets = &packets.iter().map(|v| &v[..]).collect::<Vec<_>>();
    validate_packets(&ftel, packets, &serialized[..], 1);
  }

  /// Tests if the packet splitter properly splits a message that doesn't fit 
  /// into a full packet. It should send two packets: one with the incomplete
  /// message, and a second with the XOR packet. The XOR packet should have the
  /// maximum payload size, whereas the payload size of the data packet should
  /// be whatever the message size is.
  #[test]
  fn one_unfilled_packet() {
    validate_n_unfilled_packets(1);
  }

  /// Tests if the packet splitter properly splits a message that fits into a 
  /// full packet. It should send two packets: one with the complete message, 
  /// and a second with the XOR packet. Both packets should have the maximum
  /// payload size.
  #[test]
  fn one_filled_packet() {
    validate_n_filled_packets(1);
  }

  /// See one_unfilled_packet(). Three packets should be sent here, one full,
  /// one unfilled, and the XOR.
  #[test]
  fn two_unfilled_packets() {
    validate_n_unfilled_packets(2);
  }

  /// See one_filled_packet().
  #[test]
  fn two_filled_packets() {
    validate_n_filled_packets(2);
  }

  /// See two_unfilled_packet().
  #[test]
  fn a_hundred_unfilled_packets() {
    validate_n_unfilled_packets(100);
  }

  /// See two_filled_packets().
  #[test]
  fn a_hundred_filled_packets() {
    validate_n_filled_packets(100);
  }

  /// Performs the same test as a_hundred_filled_packets(). Then, drops a packet
  /// and sees if it can be reconstructed successfully using the XOR packet.
  #[test]
  fn xor_recovery_validation_with_filled_packets() {
    xor_recovery_validation_filled(100);
  }

  /// Performs the same test as a_hundred_unfilled_packets(). Then, drops a 
  /// packet and sees if it can be reconstructed successfully using the XOR 
  /// packet.
  #[test]
  fn xor_recovery_validation_with_unfilled_packets() {
    xor_recovery_validation_unfilled(100);
  }

  /// Tests to see if the FTel socket will only send data once the deadline 
  /// passes.
  #[test]
  fn timer_test() {
    const TIMES_TO_POLL: u32 = 4;
    let deadline = TIMEOUT_DURATION * (TIMES_TO_POLL + 2);
    let (mut ftel, dest, mocket) = initialize(deadline);

    let serialized = postcard::to_allocvec(&()).unwrap();
    assert!(ftel.reverse_poll(&dest, &serialized[..]).unwrap());
    let then = Instant::now();
    assert!(recv_packet_from(&mocket).is_some());
    check_if_empty(&mocket);

    for _ in 0..TIMES_TO_POLL {
      assert!(!ftel.reverse_poll(&dest, &serialized[..]).unwrap());
      assert!(recv_packet_from(&mocket).is_none());
    }

    while Instant::now().duration_since(then) < deadline {}

    assert!(ftel.reverse_poll(&dest, &serialized[..]).unwrap());
    assert!(recv_packet_from(&mocket).is_some());
    check_if_empty(&mocket);
  }
}