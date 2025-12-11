use clap::ArgMatches;
use common::comm::{
  flight::DataMessage,
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
  borrow::Cow,
  net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket},
  thread,
  time::Duration,
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

  {
    let address: SocketAddr = "127.0.0.1:7201".parse()
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

    // randomly change the dcsp number
    let is_tel : bool = (rand::random::<u8>() % 16 == 0) || true;

    if is_tel {
      socket2.send(&raw)?;
    } else {
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
