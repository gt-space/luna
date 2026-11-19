//! The CM5 side: answers requests and safes the board when the worker goes
//! quiet.

use std::{
    io::{self, ErrorKind},
    net::{SocketAddr, UdpSocket},
    time::{Duration, Instant},
};

use common::comm::hitl::{Command, Reply, Request, Response, HEARTBEAT_TIME_LIMIT};
use jeflog::{fail, pass, warn};

use crate::hardware::Hardware;

/// How long one [`Service::poll`] waits for a request.
const POLL_TIMEOUT: Duration = Duration::from_millis(100);

/// Largest request accepted. Requests are a few bytes.
const BUFFER_SIZE: usize = 1024;

/// Runs one command against the board.
pub fn execute(hw: &mut dyn Hardware, command: Command) -> Reply {
    let result = match command {
        Command::Ping => Ok(Reply::Done),
        Command::SetDacVolts { channel, volts } => {
            hw.set_dac_volts(channel, volts).map(|()| Reply::Done)
        }
        Command::SetDigipot { channel, code } => {
            hw.set_digipot(channel, code).map(|()| Reply::Done)
        }
        Command::ReadAdcVolts { positive, negative } => {
            hw.read_adc_volts(positive, negative).map(Reply::Volts)
        }
        Command::ReadTemperature => hw.read_temperature().map(Reply::Celsius),
        Command::Safe => hw.safe().map(|()| Reply::Done),
        Command::Version => Ok(Reply::Version(crate::VERSION.to_owned())),
    };

    result.unwrap_or_else(|e| Reply::Failed(e.to_string()))
}

/// The request loop.
///
/// The watchdog only arms after the first message, so a board nobody has
/// talked to is left alone. Once armed, if no message arrives for
/// [`HEARTBEAT_TIME_LIMIT`], the board is safed once. The next message from a
/// worker re-arms it.
pub struct Service<H: Hardware> {
    socket: UdpSocket,
    hw: H,
    last_heard: Option<Instant>,
    safed: bool,
}

impl<H: Hardware> Service<H> {
    /// Listens on `address`, such as `0.0.0.0:8380`.
    pub fn bind(address: impl Into<SocketAddr>, hw: H) -> io::Result<Self> {
        let socket = UdpSocket::bind(address.into())?;
        socket.set_read_timeout(Some(POLL_TIMEOUT))?;
        Ok(Self {
            socket,
            hw,
            last_heard: None,
            safed: false,
        })
    }

    /// The address the service is listening on.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Handles at most one request, waiting up to 100 ms for it. With nothing
    /// to read, checks the watchdog instead.
    ///
    /// The watchdog is only checked when nothing is waiting. After a slow
    /// command, the pings that queued up are handled first, so the slow
    /// command does not look like silence.
    pub fn poll(&mut self) -> io::Result<()> {
        let mut buffer = [0; BUFFER_SIZE];

        match self.socket.recv_from(&mut buffer) {
            Ok((size, sender)) => self.handle(&buffer[..size], sender),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                self.check_watchdog();
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// Polls forever.
    pub fn run(&mut self) -> ! {
        loop {
            if let Err(e) = self.poll() {
                fail!("Socket error: {e}");
            }
        }
    }

    fn handle(&mut self, bytes: &[u8], sender: SocketAddr) -> io::Result<()> {
        self.last_heard = Some(Instant::now());
        if self.safed {
            pass!("Heard from {sender}; watchdog armed again.");
            self.safed = false;
        }

        let response = match postcard::from_bytes::<Request>(bytes) {
            Ok(Request { id, command }) => Response {
                id,
                reply: execute(&mut self.hw, command),
            },
            Err(e) => Response {
                id: 0,
                reply: Reply::Failed(format!("could not decode request: {e}")),
            },
        };

        let bytes = postcard::to_allocvec(&response)
            .map_err(|e| io::Error::new(ErrorKind::InvalidData, e))?;
        self.socket.send_to(&bytes, sender)?;
        Ok(())
    }

    fn check_watchdog(&mut self) {
        let Some(last_heard) = self.last_heard else {
            return;
        };

        if !self.safed && last_heard.elapsed() > HEARTBEAT_TIME_LIMIT {
            warn!(
                "No message for {} ms. Putting the board in its safe state.",
                HEARTBEAT_TIME_LIMIT.as_millis()
            );
            if let Err(e) = self.hw.safe() {
                fail!("Could not safe the board: {e}");
            }
            self.safed = true;
        }
    }
}
