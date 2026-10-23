//! FC side of the FC <-> engine controller (EC) link.
//!
//! [`FcLink`] holds the link logic with no sockets and no clock: give it each
//! received datagram and the time, and it says what to send. [`spawn`] runs
//! it on its own thread behind a UDP socket. The EC side is
//! `firmware/fc_ec_link/ec_side.c`, and the wire format is
//! `common::comm::ec_link`.
//!
//! What the link does:
//! - Opens a session with Hello, and again if the EC reboots, drops the
//!   session or goes quiet.
//! - Sends the newest demand every 10 ms, but stops once nobody has set a
//!   demand for `demand_hold`, so the EC safes itself if the FC's control side
//!   dies instead of flying on a frozen demand.
//! - Sends a heartbeat every 50 ms, separate from demands.
//! - Runs one transaction (arm, disarm, clear safing) at a time, retrying with
//!   the same ID until the EC answers. Terminate has its own slot and goes out
//!   ahead of everything, session or not.
//! - Echoes the newest EC time in every message so the EC can bound its age.

use std::{
    collections::VecDeque,
    fmt, io, mem,
    net::{SocketAddr, UdpSocket},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex, MutexGuard,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use common::comm::{
    ctv::ActuatorDemand,
    ec_link::{
        channel, Arm, Demand, EcState, Header, Hello, HelloResult, Message, Outputs, Packet,
        Reason, Source, Status, Terminate, TxnOutcome, TxnResult, MAX_DATAGRAM_LEN,
    },
};

/// Most transactions that can wait behind the one in flight.
const QUEUE_LEN: usize = 4;

/// Identity and timing for the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LinkConfig {
    /// Picked once per FC run; must be nonzero.
    pub session_id: u32,
    /// Random per FC boot.
    pub fc_boot_id: u32,
    /// ID of the current run. Placeholder until the FC run manager exists.
    pub run_id: u32,
    /// How often to send the newest demand.
    pub demand_period: Duration,
    /// How often to send a heartbeat.
    pub heartbeat_period: Duration,
    /// How often to retry Hello while not connected.
    pub hello_retry: Duration,
    /// How often to resend a transaction the EC hasn't answered.
    pub txn_retry: Duration,
    /// Sends before giving up on a transaction.
    pub txn_tries: u32,
    /// Silence from the EC after which the session is considered lost.
    pub link_timeout: Duration,
    /// How long the newest demand keeps being resent without a new one.
    pub demand_hold: Duration,
    /// How long after the echoed EC time each message stays valid.
    pub valid_for: Duration,
}

impl LinkConfig {
    /// The spec's timing: 100 Hz demands, 50 ms heartbeats and transaction
    /// retries, 350 ms link timeout.
    pub(crate) fn new(session_id: u32, fc_boot_id: u32, run_id: u32) -> Self {
        Self {
            session_id,
            fc_boot_id,
            run_id,
            demand_period: Duration::from_millis(10),
            heartbeat_period: Duration::from_millis(50),
            hello_retry: Duration::from_millis(100),
            txn_retry: Duration::from_millis(50),
            txn_tries: 5,
            link_timeout: Duration::from_millis(350),
            demand_hold: Duration::from_millis(100),
            valid_for: Duration::from_millis(50),
        }
    }
}

/// A transaction the FC can ask of the EC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Request {
    Arm,
    Disarm,
    Terminate,
    ClearSafing,
}

/// Why a request wasn't queued.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RequestError {
    /// Too many transactions already waiting.
    QueueFull,
    /// No session with the EC. Only Terminate can go out without one.
    NotConnected,
}

impl fmt::Display for RequestError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::QueueFull => write!(f, "too many EC requests already waiting"),
            Self::NotConnected => write!(f, "no session with the EC"),
        }
    }
}

/// Something about the link worth telling the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Event {
    /// A session is open.
    Connected { ec_boot_id: u32, ec_state: EcState },
    /// The EC refused Hello. Reported once per reason.
    HelloRejected(Reason),
    /// Nothing from the EC for the link timeout.
    Lost,
    /// The EC came back with a new boot ID.
    EcRebooted { ec_boot_id: u32 },
    /// The EC says our session isn't valid any more.
    SessionLost,
    /// The EC changed state.
    StateChanged {
        from: EcState,
        to: EcState,
        safing_reason: Reason,
    },
    /// The EC answered a transaction.
    Done {
        request: Request,
        outcome: TxnOutcome,
        reason: Reason,
        ec_state: EcState,
    },
    /// Retries ran out, or the session dropped after sending: the outcome is
    /// unknown, not failed.
    NoAnswer(Request),
    /// Dropped before it was ever sent, because the session dropped.
    NotSent(Request),
}

impl fmt::Display for Event {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Connected {
                ec_boot_id,
                ec_state,
            } => write!(
                f,
                "connected to EC boot {ec_boot_id:08X}, EC is {ec_state:?}"
            ),
            Self::HelloRejected(reason) => write!(f, "EC refused the session: {reason:?}"),
            Self::Lost => write!(f, "nothing from the EC in time, reconnecting"),
            Self::EcRebooted { ec_boot_id } => {
                write!(f, "EC rebooted (new boot {ec_boot_id:08X}), reconnecting")
            }
            Self::SessionLost => write!(f, "EC dropped the session, reconnecting"),
            Self::StateChanged {
                from,
                to,
                safing_reason,
            } => {
                write!(f, "EC {from:?} -> {to:?}")?;
                if *to == EcState::Safing {
                    write!(f, " ({safing_reason:?})")?;
                }
                Ok(())
            }
            Self::Done {
                request,
                outcome,
                reason,
                ec_state,
            } => match outcome {
                TxnOutcome::Accepted => write!(f, "{request:?} accepted, EC is {ec_state:?}"),
                TxnOutcome::Rejected => {
                    write!(f, "{request:?} rejected ({reason:?}), EC is {ec_state:?}")
                }
            },
            Self::NoAnswer(request) => write!(f, "no answer to {request:?}; outcome unknown"),
            Self::NotSent(request) => write!(f, "{request:?} not sent: session dropped"),
        }
    }
}

/// A transaction waiting for its answer.
#[derive(Debug, Clone, Copy)]
struct Pending {
    request: Request,
    txn_id: u32,
    tries: u32,
    next_try: Duration,
}

/// The link logic, with no sockets and no clock. Times are measured from any
/// fixed start, e.g. when the worker started.
#[derive(Debug)]
pub(crate) struct FcLink {
    config: LinkConfig,
    connected: bool,
    ec_boot_id: u32,
    ec_time_ref_us: u64,
    last_heard: Duration,
    seq: u32,
    txn_id: u32,
    next_hello: Duration,
    next_demand: Duration,
    next_heartbeat: Duration,
    demand: Option<(Demand, Duration)>,
    terminate: Option<Pending>,
    pending: Option<Pending>,
    queue: VecDeque<Request>,
    status: Option<Status>,
    last_hello_rejection: Option<Reason>,
    events: Vec<Event>,
}

impl FcLink {
    pub(crate) fn new(config: LinkConfig) -> Self {
        Self {
            config,
            connected: false,
            ec_boot_id: 0,
            ec_time_ref_us: 0,
            last_heard: Duration::ZERO,
            seq: 0,
            txn_id: 0,
            next_hello: Duration::ZERO,
            next_demand: Duration::ZERO,
            next_heartbeat: Duration::ZERO,
            demand: None,
            terminate: None,
            pending: None,
            queue: VecDeque::new(),
            status: None,
            last_hello_rejection: None,
            events: Vec::new(),
        }
    }

    /// Whether a session with the EC is open.
    pub(crate) fn connected(&self) -> bool {
        self.connected
    }

    /// The newest Status from the EC.
    pub(crate) fn status(&self) -> Option<&Status> {
        self.status.as_ref()
    }

    /// Takes the events since the last call.
    pub(crate) fn take_events(&mut self) -> Vec<Event> {
        mem::take(&mut self.events)
    }

    /// Replaces the demand to send, if it's valid. `set_at` is when the
    /// producer set it, which starts the `demand_hold` clock.
    pub(crate) fn set_demand(&mut self, demand: Demand, set_at: Duration) -> Result<(), Reason> {
        demand.check(channel::ALL)?;
        self.demand = Some((demand, set_at));
        Ok(())
    }

    /// Queues a transaction. Terminate replaces any earlier Terminate and
    /// goes out before anything else, even with no session.
    pub(crate) fn request(&mut self, request: Request) -> Result<(), RequestError> {
        if request == Request::Terminate {
            self.txn_id = self.txn_id.wrapping_add(1);
            self.terminate = Some(Pending {
                request,
                txn_id: self.txn_id,
                tries: 0,
                next_try: Duration::ZERO,
            });
            return Ok(());
        }

        if !self.connected {
            return Err(RequestError::NotConnected);
        }
        if self.queue.len() >= QUEUE_LEN {
            return Err(RequestError::QueueFull);
        }

        self.queue.push_back(request);
        Ok(())
    }

    /// Handles one datagram from the EC.
    pub(crate) fn receive(&mut self, bytes: &[u8], now: Duration) {
        let Ok(packet) = Packet::decode(bytes) else {
            return; // corrupt or not ours
        };
        if packet.message.kind().source() != Source::Ec {
            return;
        }

        let header = packet.header;
        self.last_heard = now;
        self.ec_time_ref_us = header.sent_time_us;

        match packet.message {
            Message::HelloResult(result) => self.on_hello_result(&header, &result),
            Message::TxnResult(result) => self.on_txn_result(&header, &result),
            Message::Status(status) => self.on_status(&header, status, now),
            _ => {}
        }
    }

    /// Advances the timers and returns the datagrams to send now.
    pub(crate) fn poll(&mut self, now: Duration) -> Vec<Vec<u8>> {
        let mut out = Vec::new();

        if self.connected && now.saturating_sub(self.last_heard) > self.config.link_timeout {
            self.drop_session(Event::Lost, now);
        }

        // Safing goes out first, session or not: the EC obeys it regardless.
        if let Some(pending) = self.terminate.take() {
            self.terminate = self.step_txn(pending, now, &mut out);
        }

        if !self.connected {
            if now >= self.next_hello {
                let hello = self.hello();
                self.txn_id = self.txn_id.wrapping_add(1);
                out.push(self.packet(Message::Hello(hello), self.txn_id, now));
                self.next_hello = now + self.config.hello_retry;
            }
            return out;
        }

        if self.pending.is_none() {
            if let Some(request) = self.queue.pop_front() {
                self.txn_id = self.txn_id.wrapping_add(1);
                self.pending = Some(Pending {
                    request,
                    txn_id: self.txn_id,
                    tries: 0,
                    next_try: now,
                });
            }
        }
        if let Some(pending) = self.pending.take() {
            self.pending = self.step_txn(pending, now, &mut out);
        }

        if let Some((demand, set_at)) = self.demand {
            let current = now.saturating_sub(set_at) <= self.config.demand_hold;
            if current && now >= self.next_demand {
                out.push(self.packet(Message::Demand(demand), 0, now));
                self.next_demand = advance(self.next_demand, self.config.demand_period, now);
            }
        }

        if now >= self.next_heartbeat {
            out.push(self.packet(Message::Heartbeat, 0, now));
            self.next_heartbeat = advance(self.next_heartbeat, self.config.heartbeat_period, now);
        }

        out
    }

    /// Sends `pending` if it's due, and returns it unless it's finished.
    fn step_txn(
        &mut self,
        mut pending: Pending,
        now: Duration,
        out: &mut Vec<Vec<u8>>,
    ) -> Option<Pending> {
        if now < pending.next_try {
            return Some(pending);
        }
        if pending.tries >= self.config.txn_tries {
            self.events.push(Event::NoAnswer(pending.request));
            return None;
        }

        let message = match pending.request {
            Request::Arm => Message::Arm(Arm {
                run_id: self.config.run_id,
                expected_bundle_sha256: [0; 32],
            }),
            Request::Disarm => Message::Disarm,
            Request::Terminate => Message::Terminate(Terminate {
                reason: Reason::FcTerminate,
            }),
            Request::ClearSafing => Message::ClearSafing,
        };

        out.push(self.packet(message, pending.txn_id, now));
        pending.tries += 1;
        pending.next_try = now + self.config.txn_retry;
        Some(pending)
    }

    fn hello(&self) -> Hello {
        Hello {
            fc_boot_id: self.config.fc_boot_id,
            run_id: self.config.run_id,
            demand_period_ms: millis(self.config.demand_period),
            heartbeat_period_ms: millis(self.config.heartbeat_period),
            expected_bundle_sha256: [0; 32],
        }
    }

    fn packet(&mut self, message: Message, txn_id: u32, now: Duration) -> Vec<u8> {
        self.seq = self.seq.wrapping_add(1);
        let packet = Packet {
            header: Header {
                duplicate: false,
                ec_boot_id: self.ec_boot_id,
                session_id: self.config.session_id,
                seq: self.seq,
                txn_id,
                bundle_tag: 0,
                valid_for_ms: millis(self.config.valid_for),
                sent_time_us: u64::try_from(now.as_micros()).unwrap_or(u64::MAX),
                ec_time_ref_us: self.ec_time_ref_us,
            },
            message,
        };

        let mut buf = [0; MAX_DATAGRAM_LEN];
        let len = packet
            .encode(&mut buf)
            .expect("every message fits in MAX_DATAGRAM_LEN");
        buf[..len].to_vec()
    }

    fn on_hello_result(&mut self, header: &Header, result: &HelloResult) {
        if result.outcome == TxnOutcome::Rejected {
            if self.last_hello_rejection != Some(result.reason) {
                self.events.push(Event::HelloRejected(result.reason));
                self.last_hello_rejection = Some(result.reason);
            }
            return;
        }

        if self.connected || header.session_id != self.config.session_id {
            return;
        }

        self.connected = true;
        self.ec_boot_id = header.ec_boot_id;
        self.last_hello_rejection = None;
        self.events.push(Event::Connected {
            ec_boot_id: header.ec_boot_id,
            ec_state: result.ec_state,
        });
    }

    fn on_txn_result(&mut self, header: &Header, result: &TxnResult) {
        for slot in [&mut self.terminate, &mut self.pending] {
            if let Some(pending) = slot {
                if pending.txn_id == header.txn_id {
                    self.events.push(Event::Done {
                        request: pending.request,
                        outcome: result.outcome,
                        reason: result.reason,
                        ec_state: result.ec_state,
                    });
                    *slot = None;
                }
            }
        }
    }

    fn on_status(&mut self, header: &Header, status: Status, now: Duration) {
        if self.connected {
            if header.ec_boot_id != self.ec_boot_id {
                self.drop_session(
                    Event::EcRebooted {
                        ec_boot_id: header.ec_boot_id,
                    },
                    now,
                );
            } else if !status.session_valid || header.session_id != self.config.session_id {
                self.drop_session(Event::SessionLost, now);
            }
        }

        if let Some(previous) = &self.status {
            if previous.ec_state != status.ec_state {
                self.events.push(Event::StateChanged {
                    from: previous.ec_state,
                    to: status.ec_state,
                    safing_reason: status.safing_reason,
                });
            }
        }
        self.status = Some(status);
    }

    /// Closes the session and reports anything that can no longer be sent.
    fn drop_session(&mut self, event: Event, now: Duration) {
        self.connected = false;
        self.next_hello = now;
        self.events.push(event);

        if let Some(pending) = self.pending.take() {
            self.events.push(if pending.tries > 0 {
                Event::NoAnswer(pending.request)
            } else {
                Event::NotSent(pending.request)
            });
        }
        for request in self.queue.drain(..) {
            self.events.push(Event::NotSent(request));
        }
    }
}

/// The next deadline after `next`, skipping ahead if the loop fell behind.
fn advance(next: Duration, period: Duration, now: Duration) -> Duration {
    let next = next + period;
    if next <= now {
        now + period
    } else {
        next
    }
}

fn millis(duration: Duration) -> u16 {
    u16::try_from(duration.as_millis()).unwrap_or(u16::MAX)
}

/// Converts an actuator demand into the wire demand, driving every channel.
pub(crate) fn wire_demand(demand: &ActuatorDemand) -> Demand {
    Demand {
        outputs: Outputs {
            tvc_extension_mm: demand.tvc_extension_mm.map(|v| v as f32),
            rcs_duty: demand.rcs_duty.map(|v| v as f32),
            main_throttle: demand.main_throttle as f32,
        },
        enable_mask: channel::ALL,
    }
}

/// What the rest of the FC can see of the link.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct LinkSnapshot {
    pub connected: bool,
    pub status: Option<Status>,
}

/// Handle to the link thread. Dropping it stops the thread.
pub(crate) struct EcLinkHandle {
    demand: Arc<Mutex<Option<(Demand, Instant)>>>,
    requests: mpsc::SyncSender<Request>,
    snapshot: Arc<Mutex<LinkSnapshot>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl EcLinkHandle {
    /// Replaces the demand the link sends. Rejects non-finite or out of range
    /// values instead of sending them.
    pub(crate) fn set_demand(&self, demand: &ActuatorDemand) -> Result<(), Reason> {
        let demand = wire_demand(demand);
        demand.check(channel::ALL)?;
        *lock(&self.demand) = Some((demand, Instant::now()));
        Ok(())
    }

    /// Asks the link to run a transaction. Answers are printed as they come.
    pub(crate) fn request(&self, request: Request) -> Result<(), RequestError> {
        self.requests
            .try_send(request)
            .map_err(|_| RequestError::QueueFull)
    }

    /// Whether a session is open, and the newest EC Status.
    #[cfg_attr(
        not(test),
        expect(
            dead_code,
            reason = "read once EC feedback is merged into VehicleState"
        )
    )]
    pub(crate) fn snapshot(&self) -> LinkSnapshot {
        *lock(&self.snapshot)
    }
}

impl Drop for EcLinkHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Starts the link on its own thread, listening on `bind` and talking to the
/// EC at `ec`. Only datagrams from the EC's IP are read.
pub(crate) fn spawn(
    bind: SocketAddr,
    ec: SocketAddr,
    config: LinkConfig,
) -> io::Result<EcLinkHandle> {
    let socket = UdpSocket::bind(bind)?;
    socket.set_read_timeout(Some(Duration::from_millis(1)))?;

    let demand = Arc::new(Mutex::new(None));
    let snapshot = Arc::new(Mutex::new(LinkSnapshot::default()));
    let stop = Arc::new(AtomicBool::new(false));
    let (requests, request_receiver) = mpsc::sync_channel(QUEUE_LEN);

    let thread = {
        let demand = Arc::clone(&demand);
        let snapshot = Arc::clone(&snapshot);
        let stop = Arc::clone(&stop);
        thread::Builder::new()
            .name("ec-link".into())
            .spawn(move || {
                run(
                    socket,
                    ec,
                    config,
                    &demand,
                    &request_receiver,
                    &snapshot,
                    &stop,
                )
            })?
    };

    Ok(EcLinkHandle {
        demand,
        requests,
        snapshot,
        stop,
        thread: Some(thread),
    })
}

fn run(
    socket: UdpSocket,
    ec: SocketAddr,
    config: LinkConfig,
    demand: &Mutex<Option<(Demand, Instant)>>,
    requests: &mpsc::Receiver<Request>,
    snapshot: &Mutex<LinkSnapshot>,
    stop: &AtomicBool,
) {
    let start = Instant::now();
    let mut link = FcLink::new(config);
    let mut buf = [0u8; MAX_DATAGRAM_LEN + 1];

    while !stop.load(Ordering::Relaxed) {
        // The 1 ms read timeout paces this loop when nothing arrives.
        loop {
            match socket.recv_from(&mut buf) {
                Ok((len, from)) if from.ip() == ec.ip() => {
                    link.receive(&buf[..len], start.elapsed());
                }
                Ok(_) => {}
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    break
                }
                Err(e) => {
                    eprintln!("EC link: receive failed: {e}");
                    break;
                }
            }
        }

        if let Some((new_demand, set_at)) = lock(demand).take() {
            // Already checked by the handle.
            let _ = link.set_demand(new_demand, set_at.saturating_duration_since(start));
        }

        while let Ok(request) = requests.try_recv() {
            if let Err(e) = link.request(request) {
                println!("EC link: {request:?} not sent: {e}");
            }
        }

        for datagram in link.poll(start.elapsed()) {
            if let Err(e) = socket.send_to(&datagram, ec) {
                eprintln!("EC link: send to {ec} failed: {e}");
            }
        }

        for event in link.take_events() {
            println!("EC link: {event}");
        }

        *lock(snapshot) = LinkSnapshot {
            connected: link.connected(),
            status: link.status().copied(),
        };
    }
}

#[cfg(test)]
mod tests {
    use common::comm::ec_link::Kind;

    use super::*;

    const SESSION: u32 = 0x5E55_0001;
    const EC_BOOT: u32 = 0xB007_0001;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    fn link() -> FcLink {
        FcLink::new(LinkConfig::new(SESSION, 0xF00D, 7))
    }

    fn decode_all(datagrams: &[Vec<u8>]) -> Vec<Packet> {
        datagrams
            .iter()
            .map(|d| Packet::decode(d).expect("FC sends valid datagrams"))
            .collect()
    }

    fn kinds(datagrams: &[Vec<u8>]) -> Vec<Kind> {
        decode_all(datagrams)
            .iter()
            .map(|p| p.message.kind())
            .collect()
    }

    /// An EC datagram, sent at EC time `ec_time_ms`.
    fn from_ec(message: Message, txn_id: u32, boot: u32, ec_time_ms: u64) -> Vec<u8> {
        let packet = Packet {
            header: Header {
                ec_boot_id: boot,
                session_id: SESSION,
                txn_id,
                sent_time_us: ec_time_ms * 1000,
                ..Header::default()
            },
            message,
        };
        let mut buf = [0; MAX_DATAGRAM_LEN];
        let len = packet.encode(&mut buf).unwrap();
        buf[..len].to_vec()
    }

    fn hello_result(outcome: TxnOutcome, reason: Reason) -> Message {
        Message::HelloResult(HelloResult {
            outcome,
            ec_state: EcState::Disarmed,
            reason,
            capabilities: channel::ALL,
            max_demand_age_ms: 50,
            firmware_version: 1,
            calibration_id: 0,
            link_timeout_ms: 350,
            active_bundle_sha256: [0; 32],
        })
    }

    fn status(state: EcState, session_valid: bool) -> Message {
        Message::Status(Status {
            ec_state: state,
            session_valid,
            ..Status::default()
        })
    }

    fn demand(throttle: f32) -> Demand {
        Demand {
            outputs: Outputs {
                tvc_extension_mm: [13.45, 13.45],
                rcs_duty: [0.0; 4],
                main_throttle: throttle,
            },
            enable_mask: channel::ALL,
        }
    }

    /// A link with an open session, at FC time 1 ms, having heard EC time 500 ms.
    fn connected() -> FcLink {
        let mut link = link();
        let hello = decode_all(&link.poll(ms(0)));
        let txn = hello[0].header.txn_id;
        link.receive(
            &from_ec(
                hello_result(TxnOutcome::Accepted, Reason::None),
                txn,
                EC_BOOT,
                500,
            ),
            ms(1),
        );
        link.take_events();
        link
    }

    #[test]
    fn says_hello_until_the_ec_answers() {
        let mut link = link();

        let first = decode_all(&link.poll(ms(0)));
        assert_eq!(first.len(), 1);
        let Message::Hello(hello) = first[0].message else {
            panic!("expected Hello, got {:?}", first[0].message);
        };
        assert_eq!(first[0].header.session_id, SESSION);
        assert_eq!(hello.demand_period_ms, 10);
        assert_eq!(hello.run_id, 7);

        // Nothing else goes out before the session opens, demand or not.
        link.set_demand(demand(0.5), ms(0)).unwrap();
        assert!(link.poll(ms(50)).is_empty());
        assert_eq!(kinds(&link.poll(ms(100))), [Kind::Hello]);

        let txn = Packet::decode(&link.poll(ms(200))[0])
            .unwrap()
            .header
            .txn_id;
        link.receive(
            &from_ec(
                hello_result(TxnOutcome::Accepted, Reason::None),
                txn,
                EC_BOOT,
                500,
            ),
            ms(201),
        );
        assert!(link.connected());
        assert_eq!(
            link.take_events(),
            [Event::Connected {
                ec_boot_id: EC_BOOT,
                ec_state: EcState::Disarmed
            }]
        );
    }

    #[test]
    fn a_refused_hello_is_reported_once_and_retried() {
        let mut link = link();
        link.poll(ms(0));
        let refused = hello_result(TxnOutcome::Rejected, Reason::WrongState);
        link.receive(&from_ec(refused, 1, EC_BOOT, 500), ms(1));
        link.receive(&from_ec(refused, 2, EC_BOOT, 600), ms(101));

        assert!(!link.connected());
        assert_eq!(
            link.take_events(),
            [Event::HelloRejected(Reason::WrongState)]
        );
        assert_eq!(kinds(&link.poll(ms(101))), [Kind::Hello]);
    }

    #[test]
    fn sends_demands_at_100_hz_and_heartbeats_at_20_hz() {
        let mut link = connected();
        link.set_demand(demand(0.5), ms(1)).unwrap();

        let mut sent = Vec::new();
        for t in 1..=60 {
            sent.extend(decode_all(&link.poll(ms(t))));
            if t % 10 == 0 {
                link.set_demand(demand(0.5), ms(t)).unwrap();
            }
        }

        let demands = sent
            .iter()
            .filter(|p| p.message.kind() == Kind::Demand)
            .count();
        let heartbeats = sent
            .iter()
            .filter(|p| p.message.kind() == Kind::Heartbeat)
            .count();
        assert_eq!(demands, 7, "at 1, 10, 20, 30, 40, 50, 60 ms");
        assert_eq!(heartbeats, 2, "at 1 and 50 ms");

        // Every message carries the session, the EC's boot and the newest EC time.
        for packet in &sent {
            assert_eq!(packet.header.session_id, SESSION);
            assert_eq!(packet.header.ec_boot_id, EC_BOOT);
            assert_eq!(packet.header.ec_time_ref_us, 500_000);
            assert_eq!(packet.header.valid_for_ms, 50);
        }
        let seqs: Vec<u32> = sent.iter().map(|p| p.header.seq).collect();
        assert!(seqs.windows(2).all(|w| w[1] == w[0] + 1));
    }

    #[test]
    fn stops_resending_a_demand_nobody_refreshes() {
        let mut link = connected();
        link.set_demand(demand(0.5), ms(1)).unwrap();

        let mut last_demand = None;
        for t in 1..=300 {
            if kinds(&link.poll(ms(t))).contains(&Kind::Demand) {
                last_demand = Some(t);
            }
            link.receive(
                &from_ec(status(EcState::Armed, true), 0, EC_BOOT, 500 + t),
                ms(t),
            );
        }

        // Set at 1 ms and held for 100 ms: the last one on the 10 ms grid is at
        // 100 ms.
        assert_eq!(last_demand, Some(100));
    }

    #[test]
    fn rejects_invalid_demands_up_front() {
        let mut link = connected();
        assert_eq!(
            link.set_demand(demand(f32::NAN), ms(1)),
            Err(Reason::NonFinite)
        );
        assert_eq!(link.set_demand(demand(1.5), ms(1)), Err(Reason::OutOfRange));
        assert!(!kinds(&link.poll(ms(2))).contains(&Kind::Demand));
    }

    #[test]
    fn retries_a_transaction_with_the_same_id_until_answered() {
        let mut link = connected();
        link.request(Request::Arm).unwrap();

        let first = decode_all(&link.poll(ms(2)));
        let arm: Vec<&Packet> = first
            .iter()
            .filter(|p| p.message.kind() == Kind::Arm)
            .collect();
        assert_eq!(arm.len(), 1);
        let txn = arm[0].header.txn_id;

        assert!(!kinds(&link.poll(ms(30))).contains(&Kind::Arm));
        let retry = decode_all(&link.poll(ms(52)));
        let retried: Vec<&Packet> = retry
            .iter()
            .filter(|p| p.message.kind() == Kind::Arm)
            .collect();
        assert_eq!(retried.len(), 1);
        assert_eq!(retried[0].header.txn_id, txn);

        let answer = Message::TxnResult(TxnResult {
            request_kind: Kind::Arm,
            outcome: TxnOutcome::Accepted,
            reason: Reason::None,
            ec_state: EcState::Armed,
        });
        link.receive(&from_ec(answer, txn, EC_BOOT, 560), ms(60));
        assert_eq!(
            link.take_events(),
            [Event::Done {
                request: Request::Arm,
                outcome: TxnOutcome::Accepted,
                reason: Reason::None,
                ec_state: EcState::Armed
            }]
        );
        assert!(!kinds(&link.poll(ms(200))).contains(&Kind::Arm));
    }

    #[test]
    fn gives_up_after_five_tries_and_says_the_outcome_is_unknown() {
        let mut link = connected();
        link.request(Request::Disarm).unwrap();

        let mut sends = 0;
        for t in (2..=400).step_by(10) {
            sends += kinds(&link.poll(ms(t)))
                .iter()
                .filter(|k| **k == Kind::Disarm)
                .count();
            link.receive(
                &from_ec(status(EcState::Armed, true), 0, EC_BOOT, 500 + t),
                ms(t),
            );
        }

        assert_eq!(sends, 5);
        assert_eq!(link.take_events(), [Event::NoAnswer(Request::Disarm)]);
    }

    #[test]
    fn only_terminate_goes_out_without_a_session() {
        let mut link = link();
        assert_eq!(link.request(Request::Arm), Err(RequestError::NotConnected));
        link.request(Request::Terminate).unwrap();

        let sent = kinds(&link.poll(ms(0)));
        assert_eq!(sent, [Kind::Terminate, Kind::Hello]);
    }

    #[test]
    fn terminate_jumps_ahead_of_a_pending_transaction() {
        let mut link = connected();
        link.request(Request::Arm).unwrap();
        link.request(Request::Disarm).unwrap();
        link.request(Request::Terminate).unwrap();

        let sent = kinds(&link.poll(ms(2)));
        assert_eq!(sent[0], Kind::Terminate);
        assert!(sent.contains(&Kind::Arm));
        assert!(
            !sent.contains(&Kind::Disarm),
            "one ordinary transaction at a time"
        );
    }

    #[test]
    fn the_queue_is_bounded() {
        let mut link = connected();
        for _ in 0..QUEUE_LEN {
            link.request(Request::Disarm).unwrap();
        }
        assert_eq!(link.request(Request::Disarm), Err(RequestError::QueueFull));
    }

    #[test]
    fn silence_drops_the_session_and_reports_what_was_lost() {
        let mut link = connected(); // last heard at 1 ms
        link.request(Request::Arm).unwrap();
        link.request(Request::Disarm).unwrap();
        link.poll(ms(2)); // Arm sent once

        assert!(link.connected());
        link.poll(ms(351));
        assert!(link.connected());

        let sent = kinds(&link.poll(ms(352)));
        assert!(!link.connected());
        assert_eq!(sent, [Kind::Hello]);
        assert_eq!(
            link.take_events(),
            [
                Event::Lost,
                Event::NoAnswer(Request::Arm),
                Event::NotSent(Request::Disarm)
            ]
        );
    }

    #[test]
    fn a_new_boot_id_or_a_dropped_session_means_hello_again() {
        let mut link = connected();
        link.receive(
            &from_ec(status(EcState::BootSafe, false), 0, EC_BOOT + 1, 10),
            ms(5),
        );
        assert!(!link.connected());
        assert_eq!(
            link.take_events(),
            [Event::EcRebooted {
                ec_boot_id: EC_BOOT + 1
            }]
        );
        assert_eq!(kinds(&link.poll(ms(5))), [Kind::Hello]);

        let mut link = connected();
        link.receive(
            &from_ec(status(EcState::Safing, false), 0, EC_BOOT, 900),
            ms(5),
        );
        assert!(!link.connected());
        assert_eq!(link.take_events(), [Event::SessionLost]);
    }

    #[test]
    fn reports_ec_state_changes() {
        let mut link = connected();
        link.receive(
            &from_ec(status(EcState::Disarmed, true), 0, EC_BOOT, 510),
            ms(2),
        );
        link.receive(
            &from_ec(status(EcState::Armed, true), 0, EC_BOOT, 520),
            ms(3),
        );
        let safing = Message::Status(Status {
            ec_state: EcState::Safing,
            session_valid: true,
            safing_reason: Reason::DemandTimeout,
            ..Status::default()
        });
        link.receive(&from_ec(safing, 0, EC_BOOT, 530), ms(4));

        assert_eq!(
            link.take_events(),
            [
                Event::StateChanged {
                    from: EcState::Disarmed,
                    to: EcState::Armed,
                    safing_reason: Reason::None
                },
                Event::StateChanged {
                    from: EcState::Armed,
                    to: EcState::Safing,
                    safing_reason: Reason::DemandTimeout
                }
            ]
        );
        assert_eq!(link.status().unwrap().ec_state, EcState::Safing);
    }

    #[test]
    fn ignores_garbage() {
        let mut link = connected();
        link.receive(&[0xFF; 60], ms(2));
        assert!(link.connected());
        assert!(link.take_events().is_empty());
        assert!(link.status().is_none());
    }

    /// Runs the real link thread against the C fake EC over UDP on localhost.
    /// Build the fake EC (`make fake_ec` in `firmware/fc_ec_link`), then:
    ///
    /// ```text
    /// FAKE_EC=../firmware/fc_ec_link/fake_ec \
    ///     cargo test -p ctv-flight-computer against_fake_ec -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "needs the fake EC binary; see the doc comment"]
    fn against_fake_ec() {
        use std::process::{Command, Stdio};

        let Ok(fake_ec) = std::env::var("FAKE_EC") else {
            panic!("set FAKE_EC to the fake_ec binary");
        };
        let mut child = Command::new(&fake_ec)
            .arg("-p")
            .arg("18378")
            .stdout(Stdio::null())
            .spawn()
            .expect("start the fake EC");
        thread::sleep(ms(200));

        let result = std::panic::catch_unwind(|| {
            let ec: SocketAddr = "127.0.0.1:18378".parse().unwrap();
            let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
            let handle = spawn(bind, ec, LinkConfig::new(SESSION, 0xF00D, 7)).unwrap();

            let wait_for = |what: &str, ok: &dyn Fn(&LinkSnapshot) -> bool, feed: Option<f64>| {
                let deadline = Instant::now() + ms(1000);
                while Instant::now() < deadline {
                    if let Some(throttle) = feed {
                        let d = ActuatorDemand {
                            tvc_extension_mm: [20.0, 20.0],
                            rcs_duty: [0.1, 0.2, 0.3, 0.4],
                            main_throttle: throttle,
                        };
                        handle.set_demand(&d).unwrap();
                    }
                    if ok(&handle.snapshot()) {
                        return;
                    }
                    thread::sleep(ms(10));
                }
                panic!("timed out waiting for {what}: {:?}", handle.snapshot());
            };
            let state = |s: &LinkSnapshot| s.status.map(|s| s.ec_state);

            wait_for(
                "a session",
                &|s| s.connected && state(s) == Some(EcState::Disarmed),
                None,
            );

            // Demands flow first, then Arm, like a real sequence.
            wait_for(
                "demands stored",
                &|s| s.status.is_some_and(|s| s.last_accepted_demand_seq > 0),
                Some(0.5),
            );
            handle.request(Request::Arm).unwrap();
            wait_for(
                "demands applied",
                &|s| {
                    s.status.is_some_and(|s| {
                        s.ec_state == EcState::Armed && s.applied.main_throttle == 0.5
                    })
                },
                Some(0.5),
            );

            // Stop feeding: after the 100 ms hold the FC stops sending, and the
            // EC safes itself 30 ms later.
            wait_for(
                "demand timeout",
                &|s| {
                    s.status.is_some_and(|s| {
                        s.ec_state == EcState::Safing && s.safing_reason == Reason::DemandTimeout
                    })
                },
                None,
            );

            handle.request(Request::ClearSafing).unwrap();
            wait_for(
                "clear safing",
                &|s| state(s) == Some(EcState::Disarmed),
                Some(0.2),
            );
            handle.request(Request::Arm).unwrap();
            wait_for(
                "re-armed",
                &|s| {
                    s.status.is_some_and(|s| {
                        s.ec_state == EcState::Armed && s.applied.main_throttle == 0.2
                    })
                },
                Some(0.2),
            );

            handle.request(Request::Terminate).unwrap();
            wait_for(
                "terminate",
                &|s| {
                    s.status.is_some_and(|s| {
                        s.ec_state == EcState::Safing && s.safing_reason == Reason::FcTerminate
                    })
                },
                Some(0.2),
            );

            let status = handle.snapshot().status.unwrap();
            assert_eq!(status.counters.crc_errors, 0);
            assert_eq!(status.counters.stale, 0, "fresh demands only");
        });

        let _ = child.kill();
        let _ = child.wait();
        if let Err(panic) = result {
            std::panic::resume_unwind(panic);
        }
    }
}
