use axum::extract::ws::close_code::SIZE;
use clap::ArgMatches;
use common::comm::{
  flight::{DataMessage, FTEL_MTU_TRANSMISSON_LENGTH, FTEL_PACKET_PAYLOAD_LENGTH, PACKET_ID_INDEX, SIZE_RANGE, STATE_ID_INDEX, TOTAL_INDEX},
  sam::{ChannelType, DataPoint, Unit},
  CompositeValveState,
  Measurement,
  Statistics,
  ValveState,
  VehicleState,
};

use socket2::{self, Domain, Socket, Type};

use jeflog::fail;
use std::{
  borrow::Cow, collections::VecDeque, net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket}, thread, time::Duration
};

pub fn emulate_flight() -> anyhow::Result<()> {
  let _flight = TcpStream::connect("localhost:5025")?;

  let socket = Socket::new(Domain::IPV4, Type::DGRAM, None)?;
  {
    let address: SocketAddr = "127.0.0.1:7201".parse()
      .expect("If this blows up I do too");
    socket.connect(&address.into())?;
  }
  
  let socket2 = Socket::new(Domain::IPV4, Type::DGRAM, None)?;

  let mut tel_state_id : u8 = 0;

  {
    let address: SocketAddr = "127.0.0.1:7202".parse()
      .expect("If this blows up I do too");
    socket2.connect(&address.into())?;
  }
  socket2.set_tos_v4(0xF << 2)?;

  let mut mock_vehicle_state = VehicleState::new();
  mock_vehicle_state.valve_states.insert(
    "BBV".to_owned(),
    CompositeValveState {
      commanded: ValveState::Closed,
      actual: ValveState::Closed,
    },
  );
  mock_vehicle_state.valve_states.insert(
    "SWV".to_owned(),
    CompositeValveState {
      commanded: ValveState::Open,
      actual: ValveState::Open,
    },
  );
  mock_vehicle_state.valve_states.insert(
    "BYE".to_owned(),
    CompositeValveState {
      commanded: ValveState::Closed,
      actual: ValveState::Disconnected,
    },
  );
  mock_vehicle_state.valve_states.insert(
    "HUH".to_owned(),
    CompositeValveState {
      commanded: ValveState::Open,
      actual: ValveState::Undetermined,
    },
  );
  mock_vehicle_state.valve_states.insert(
    "BAD".to_owned(),
    CompositeValveState {
      commanded: ValveState::Closed,
      actual: ValveState::Fault,
    },
  );

  mock_vehicle_state.rolling.insert(
    String::from("sam-01"),
    Statistics {
      rolling_average: Duration::from_secs_f64(
        5.0 + rand::random::<f64>() * 5.0,
      ),
      time_since_last_update: 2.5 + rand::random::<f64>() * 2.5,
      delta_time: Duration::from_millis(5),
    },
  );

  let mut raw = postcard::to_allocvec(&mock_vehicle_state)?;
  postcard::from_bytes::<VehicleState>(&raw).unwrap();

  loop {
    let stats = mock_vehicle_state.rolling.get_mut("sam-01").unwrap();
    stats.delta_time =
      Duration::from_secs_f64((5.0 + rand::random::<f64>() * 5.0) / 1000.0);
    stats.time_since_last_update = (2.5 + rand::random::<f64>() * 2.5) / 1000.0;
    stats.rolling_average =
      Duration::from_secs_f64((5.0 + rand::random::<f64>() * 5.0) / 1000.0);

    mock_vehicle_state.sensor_readings.insert(
      "KBPT".to_owned(),
      Measurement {
        value: rand::random::<f64>() * 120.0,
        unit: Unit::Psi,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "WTPT".to_owned(),
      Measurement {
        value: rand::random::<f64>() * 1000.0,
        unit: Unit::Psi,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "BBV_V".to_owned(),
      Measurement {
        value: 2.2,
        unit: Unit::Volts,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "BBV_I".to_owned(),
      Measurement {
        value: 0.01,
        unit: Unit::Amps,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "SWV_V".to_owned(),
      Measurement {
        value: 24.0,
        unit: Unit::Volts,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "SWV_I".to_owned(),
      Measurement {
        value: 0.10,
        unit: Unit::Amps,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "BAD_V".to_owned(),
      Measurement {
        value: 1000.0,
        unit: Unit::Volts,
      },
    );
    mock_vehicle_state.sensor_readings.insert(
      "BAD_I".to_owned(),
      Measurement {
        value: 0.0,
        unit: Unit::Amps,
      },
    );
    raw = postcard::to_allocvec(&mock_vehicle_state)?;
    let mut raw_check = Vec::<u8>::new();

    // randomly change the dcsp number
    let is_tel : bool = (rand::random::<u8>() % 4 == 0);

    if is_tel {
      let size = raw.len();
      let mut index = 0;
      let mut packet_id = 0;

      let total_index : u8 = 1 + ((size + FTEL_PACKET_PAYLOAD_LENGTH - 1) / FTEL_PACKET_PAYLOAD_LENGTH) as u8;
      let mut xor_checksum_packet : [u8; FTEL_MTU_TRANSMISSON_LENGTH] = [0; FTEL_MTU_TRANSMISSON_LENGTH];
      let (xor_metadata, xor_buffer) = xor_checksum_packet.split_at_mut(SIZE_RANGE.end);

      while index < size {
        let mut end = if size > index + FTEL_PACKET_PAYLOAD_LENGTH { 
          index + FTEL_PACKET_PAYLOAD_LENGTH 
        } else { 
          size
        };

        let mut packet : [u8; FTEL_MTU_TRANSMISSON_LENGTH] = [0; FTEL_MTU_TRANSMISSON_LENGTH];
        let (metadata, buffer) = packet.split_at_mut(SIZE_RANGE.end);

        metadata[STATE_ID_INDEX] = tel_state_id;
        metadata[PACKET_ID_INDEX] = packet_id;
        metadata[TOTAL_INDEX] = total_index;
        metadata[SIZE_RANGE].copy_from_slice(&(size as u16).to_be_bytes());

        buffer[0..end-index].copy_from_slice(&raw[index..end]);

        for byte_index in 0..FTEL_PACKET_PAYLOAD_LENGTH {
          xor_buffer[byte_index] = xor_buffer[byte_index] ^ buffer[byte_index];
        }

        raw_check.extend_from_slice(&buffer[..]);
        
        // random drops
        if (rand::random::<u8>() % 8 != 0) {
          socket2.send(&packet[..])?;
        }

        index += FTEL_PACKET_PAYLOAD_LENGTH;
        packet_id += 1;
      }
      
      xor_metadata[STATE_ID_INDEX] = tel_state_id;
      xor_metadata[PACKET_ID_INDEX] = packet_id;
      xor_metadata[TOTAL_INDEX] = total_index;
      xor_metadata[SIZE_RANGE].copy_from_slice(&(size as u16).to_be_bytes());

      raw_check.extend_from_slice(&xor_buffer[..]);
      
      // random drops
      if (rand::random::<u8>() % 8 != 0) {
        socket2.send(&xor_checksum_packet)?;
      }

      tel_state_id = tel_state_id.wrapping_add(1);


      let read_size : usize = u16::from_be_bytes(xor_checksum_packet[SIZE_RANGE].try_into().unwrap()) as usize;
      let read_packet_id : usize = xor_checksum_packet[PACKET_ID_INDEX] as usize;
      let read_total : usize = xor_checksum_packet[TOTAL_INDEX] as usize;
      // ensure data is valid
      assert!(read_size == size);
      print!("TEL ");
      let state = postcard::from_bytes::<VehicleState>(
        &raw_check[..read_size],
      )?;
    } else {
      print!("STD ");
      socket.send(&raw)?;
    }
    thread::sleep(Duration::from_millis(10));
  }
}

pub fn emulate_sam(flight: SocketAddr) -> anyhow::Result<()> {
  let socket = UdpSocket::bind("0.0.0.0:0")?;
  socket.connect(flight)?;

  let mut buffer = [0; 1024];
  let data_points = vec![
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::CurrentLoop,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::RailVoltage,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::RailCurrent,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::Rtd,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::DifferentialSignal,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::Tc,
    },
    DataPoint {
      value: 23.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::ValveVoltage,
    },
    DataPoint {
      value: 0.00,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::ValveCurrent,
    },
  ];

  let board_id = "sam-01";

  let identity = DataMessage::Identity(board_id.to_owned());
  let handshake = postcard::to_slice(&identity, &mut buffer)?;
  socket.send(handshake)?;

  loop {
    let message =
      DataMessage::Sam(board_id.to_owned(), Cow::Borrowed(&data_points));

    let serialized = postcard::to_slice(&message, &mut buffer)?;
    socket.send(serialized)?;

    thread::sleep(Duration::from_millis(1));
  }
}

/// Tool function which emulates different components of the software stack.
pub fn emulate(args: &ArgMatches) -> anyhow::Result<()> {
  let component = args.get_one::<String>("component").unwrap();

  match component.as_str() {
    "flight" => emulate_flight(),
    "sam" => emulate_sam(
      "localhost:4573"
        .to_socket_addrs()?
        .find(|addr| addr.is_ipv4())
        .unwrap(),
    ),
    other => {
      fail!("Unrecognized emulator component '{other}'.");
      Ok(())
    }
  }
}
