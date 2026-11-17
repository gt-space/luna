//! Runs the service on localhost against the mock board and talks to it with
//! the client.

use std::{
    net::UdpSocket,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use common::comm::hitl::{Command, Reply, Response, HEARTBEAT_TIME_LIMIT};
use hitl::{
    client::{Client, Error},
    hardware::{Mock, DIGIPOT_SAFE},
    service::{execute, Service},
};

/// A service running in a thread until dropped.
struct Running {
    address: std::net::SocketAddr,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Running {
    fn start(mock: Mock) -> Self {
        let mut service = Service::bind(([127, 0, 0, 1], 0), mock).unwrap();
        let address = service.local_addr().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let stop = stop.clone();
            thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    service.poll().unwrap();
                }
            })
        };
        Self {
            address,
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[test]
fn execute_checks_channels_and_answers() {
    let mut mock = Mock::default();

    assert_eq!(execute(&mut mock, Command::Ping), Reply::Done);
    assert_eq!(
        execute(
            &mut mock,
            Command::SetDacVolts {
                channel: 2,
                volts: 1.25
            }
        ),
        Reply::Done
    );
    assert_eq!(mock.state.lock().unwrap().dac_volts[2], 1.25);

    assert!(matches!(
        execute(
            &mut mock,
            Command::SetDacVolts {
                channel: 8,
                volts: 1.0
            }
        ),
        Reply::Failed(_)
    ));
    assert!(matches!(
        execute(
            &mut mock,
            Command::SetDigipot {
                channel: 4,
                code: 0
            }
        ),
        Reply::Failed(_)
    ));
    assert!(matches!(
        execute(
            &mut mock,
            Command::ReadAdcVolts {
                positive: 10,
                negative: None
            }
        ),
        Reply::Failed(_)
    ));
    assert_eq!(
        execute(&mut mock, Command::ReadTemperature),
        Reply::Celsius(Mock::TEMPERATURE)
    );

    assert_eq!(execute(&mut mock, Command::Safe), Reply::Done);
    assert_eq!(mock.state.lock().unwrap().dac_volts, [0.0; 8]);
}

#[test]
fn client_round_trip() {
    let mock = Mock::default();
    let service = Running::start(mock.clone());
    let mut client = Client::connect(service.address).unwrap();

    client.ping().unwrap();
    client.set_dac_volts(0, 2.0).unwrap();
    client.set_digipot(3, 0x40).unwrap();
    assert_eq!(client.read_adc_volts(1, None).unwrap(), 0.0);
    assert_eq!(client.read_temperature().unwrap(), Mock::TEMPERATURE);

    {
        let state = mock.state.lock().unwrap();
        assert_eq!(state.dac_volts[0], 2.0);
        assert_eq!(state.digipot_codes[3], 0x40);
    }

    // A driver error comes back as an error, not a hang.
    assert!(matches!(
        client.set_dac_volts(9, 1.0),
        Err(Error::Failed(_))
    ));
}

#[test]
fn garbage_gets_a_failed_reply() {
    let service = Running::start(Mock::default());
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_secs(1)))
        .unwrap();

    socket.send_to(&[0xFF; 4], service.address).unwrap();
    let mut buffer = [0; 1024];
    let size = socket.recv(&mut buffer).unwrap();
    let response: Response = postcard::from_bytes(&buffer[..size]).unwrap();

    assert_eq!(response.id, 0);
    assert!(matches!(response.reply, Reply::Failed(_)));
}

#[test]
fn heartbeat_keeps_the_board_live_and_dropping_the_client_safes_it() {
    let mock = Mock::default();
    let service = Running::start(mock.clone());
    let mut client = Client::connect(service.address).unwrap();

    client.set_dac_volts(5, 1.5).unwrap();
    client.set_digipot(1, 0x10).unwrap();

    // Longer than the limit: the heartbeat alone must keep the board live.
    thread::sleep(HEARTBEAT_TIME_LIMIT * 2);
    assert_eq!(mock.state.lock().unwrap().safe_count, 0);
    assert_eq!(mock.state.lock().unwrap().dac_volts[5], 1.5);

    drop(client);

    let deadline = Instant::now() + HEARTBEAT_TIME_LIMIT * 3;
    while mock.state.lock().unwrap().safe_count == 0 {
        assert!(Instant::now() < deadline, "board was never safed");
        thread::sleep(Duration::from_millis(50));
    }

    let state = mock.state.lock().unwrap();
    assert_eq!(state.dac_volts, [0.0; 8]);
    assert_eq!(state.digipot_codes, [DIGIPOT_SAFE; 4]);
    assert_eq!(state.safe_count, 1, "safed once, not on every poll");
}

#[test]
fn silent_board_is_never_safed() {
    let mock = Mock::default();
    let _service = Running::start(mock.clone());

    thread::sleep(HEARTBEAT_TIME_LIMIT * 2);
    assert_eq!(mock.state.lock().unwrap().safe_count, 0);
}
