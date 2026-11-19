//! The Meerkat side: sends commands to the CM5 service and keeps it alive.

use std::{
    fmt, io,
    net::{SocketAddr, ToSocketAddrs, UdpSocket},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use common::comm::hitl::{Command, Reply, Request, Response, HEARTBEAT_INTERVAL};

/// How long to wait for each reply before sending again.
const REPLY_TIMEOUT: Duration = Duration::from_millis(500);

/// How many times a request is sent before giving up.
const ATTEMPTS: u32 = 3;

/// Largest response accepted.
const BUFFER_SIZE: usize = 1024;

/// Why a request did not get the reply it wanted.
#[derive(Debug)]
pub enum Error {
    /// No reply after every attempt.
    Timeout,
    /// The socket failed.
    Io(io::Error),
    /// A reply came back that could not be decoded.
    Decode(postcard::Error),
    /// The service ran the command and it failed, with the service's reason.
    Failed(String),
    /// The service answered with a reply that does not fit the command.
    Unexpected(Reply),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => write!(f, "no reply from the CM5 after {ATTEMPTS} attempts"),
            Self::Io(e) => write!(f, "socket error: {e}"),
            Self::Decode(e) => write!(f, "could not decode reply: {e}"),
            Self::Failed(reason) => write!(f, "CM5 reported: {reason}"),
            Self::Unexpected(reply) => write!(f, "unexpected reply: {reply:?}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// Result type for client calls.
pub type Result<T> = std::result::Result<T, Error>;

/// A connection to the CM5 service.
///
/// While it exists, a background thread pings the service every
/// [`HEARTBEAT_INTERVAL`]. Dropping it (or the test process dying) stops the
/// pings, and the CM5 safes the board within a second.
pub struct Client {
    socket: UdpSocket,
    next_id: u32,
    running: Arc<AtomicBool>,
    heartbeat: Option<JoinHandle<()>>,
}

impl Client {
    /// Connects to the service at `address`, such as `hitl-cm5.local:8380`,
    /// and starts the heartbeat.
    pub fn connect(address: impl ToSocketAddrs) -> Result<Self> {
        let address = address
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "address did not resolve"))?;

        let socket = bind_to(address)?;
        socket.set_read_timeout(Some(REPLY_TIMEOUT))?;

        // The heartbeat gets its own socket so its replies never mix with
        // the replies to commands.
        let heartbeat_socket = bind_to(address)?;
        let running = Arc::new(AtomicBool::new(true));
        let heartbeat = {
            let running = running.clone();
            thread::spawn(move || heartbeat(heartbeat_socket, running))
        };

        Ok(Self {
            socket,
            next_id: 1,
            running,
            heartbeat: Some(heartbeat),
        })
    }

    /// Sends a command and waits for its reply, sending again if none comes.
    ///
    /// Every command is safe to repeat: setting an output twice leaves it the
    /// same, and a read just reads again.
    pub fn request(&mut self, command: Command) -> Result<Reply> {
        let id = self.next_id;
        // Skip 0, which the service uses for requests it could not decode.
        self.next_id = self.next_id.checked_add(1).unwrap_or(1);

        let request = postcard::to_allocvec(&Request { id, command }).map_err(Error::Decode)?;
        let mut buffer = [0; BUFFER_SIZE];

        for _ in 0..ATTEMPTS {
            self.socket.send(&request)?;

            // Skip stale replies to earlier attempts until this one's arrives
            // or the wait runs out.
            loop {
                match self.socket.recv(&mut buffer) {
                    Ok(size) => {
                        let response = postcard::from_bytes::<Response>(&buffer[..size])
                            .map_err(Error::Decode)?;
                        if response.id == id {
                            return match response.reply {
                                Reply::Failed(reason) => Err(Error::Failed(reason)),
                                reply => Ok(reply),
                            };
                        }
                    }
                    Err(e)
                        if matches!(
                            e.kind(),
                            io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                        ) =>
                    {
                        break
                    }
                    Err(e) => return Err(e.into()),
                }
            }
        }

        Err(Error::Timeout)
    }

    /// Checks that the service answers.
    pub fn ping(&mut self) -> Result<()> {
        self.done(Command::Ping)
    }

    /// Drives one DAC channel (0 to 7) to a voltage.
    pub fn set_dac_volts(&mut self, channel: u8, volts: f64) -> Result<()> {
        self.done(Command::SetDacVolts { channel, volts })
    }

    /// Sets one digipot channel (0 to 3) to a wiper code.
    pub fn set_digipot(&mut self, channel: u8, code: u8) -> Result<()> {
        self.done(Command::SetDigipot { channel, code })
    }

    /// Reads the voltage between two ADC inputs (0 to 9). `None` for the
    /// negative input measures against AINCOM, which is ground.
    pub fn read_adc_volts(&mut self, positive: u8, negative: Option<u8>) -> Result<f64> {
        match self.request(Command::ReadAdcVolts { positive, negative })? {
            Reply::Volts(volts) => Ok(volts),
            reply => Err(Error::Unexpected(reply)),
        }
    }

    /// Reads the board temperature in degrees Celsius.
    pub fn read_temperature(&mut self) -> Result<f64> {
        match self.request(Command::ReadTemperature)? {
            Reply::Celsius(celsius) => Ok(celsius),
            reply => Err(Error::Unexpected(reply)),
        }
    }

    /// Asks which build the service is running: a git commit, or `dev`.
    pub fn version(&mut self) -> Result<String> {
        match self.request(Command::Version)? {
            Reply::Version(version) => Ok(version),
            reply => Err(Error::Unexpected(reply)),
        }
    }

    /// Puts the board in its safe state now.
    pub fn safe(&mut self) -> Result<()> {
        self.done(Command::Safe)
    }

    fn done(&mut self, command: Command) -> Result<()> {
        match self.request(command)? {
            Reply::Done => Ok(()),
            reply => Err(Error::Unexpected(reply)),
        }
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(heartbeat) = self.heartbeat.take() {
            let _ = heartbeat.join();
        }
    }
}

/// A socket on any free port, connected to `address`.
fn bind_to(address: SocketAddr) -> io::Result<UdpSocket> {
    let local: SocketAddr = if address.is_ipv4() {
        ([0, 0, 0, 0], 0).into()
    } else {
        ([0u16; 8], 0).into()
    };
    let socket = UdpSocket::bind(local)?;
    socket.connect(address)?;
    Ok(socket)
}

/// Pings until `running` goes false. Replies are drained and ignored; the ping
/// only has to reach the service.
fn heartbeat(socket: UdpSocket, running: Arc<AtomicBool>) {
    let _ = socket.set_nonblocking(true);
    let ping = postcard::to_allocvec(&Request {
        id: 0,
        command: Command::Ping,
    })
    .expect("a ping always encodes");
    let mut buffer = [0; BUFFER_SIZE];

    while running.load(Ordering::Relaxed) {
        let _ = socket.send(&ping);
        while socket.recv(&mut buffer).is_ok() {}
        thread::sleep(HEARTBEAT_INTERVAL);
    }
}
