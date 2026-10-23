//! Nonblocking scheduler for two independent half-duplex PA-12 ports.
//!
//! A caller repeatedly asks for ready transmissions, starts UART DMA/IT, then
//! reports TX completion and parsed reply results. No method waits for time or
//! UART activity. Each port permits one active transaction; the other port
//! remains schedulable while the first waits for a reply.

use crate::Packet;

/// Minimum quiet time after a command write, from the PA-12 link guidance.
pub const WRITE_GAP_MS: u32 = 5;
/// Minimum quiet time after a reply/read, from the PA-12 link guidance.
pub const READ_GAP_MS: u32 = 10;
/// Minimum spacing between configuration transactions on one port.
pub const CONFIG_INTERVAL_MS: u32 = 1_000;
/// Backoff before a retry; configuration retries remain subject to the
/// longer per-port configuration rate limit.
pub const RETRY_BACKOFF_MS: u32 = 100;
/// Includes the first attempt: no transaction is sent more than twice.
pub const MAX_ATTEMPTS: u8 = 2;

/// One of the two physically independent half-duplex links.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    /// PA-12 actuator 1, UART7.
    A1,
    /// PA-12 actuator 2, UART5.
    A2,
}

impl Port {
    const fn index(self) -> usize {
        match self {
            Self::A1 => 0,
            Self::A2 => 1,
        }
    }
}

/// Scheduler priority. Motion includes both normal goals and safing goals.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    /// Highest priority; a queued goal replaces an older unsent goal.
    Motion,
    /// Optional position, status, or diagnostic reads.
    Diagnostic,
    /// Configuration operations; limited to one per second per port.
    Configuration,
}

/// A packet ready for the caller to transmit asynchronously.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transmission {
    /// Independent UART to use.
    pub port: Port,
    /// Encoded PA-12 command bytes.
    pub packet: Packet,
    /// Work category, useful for logging.
    pub class: Class,
    /// One-based attempt number.
    pub attempt: u8,
    /// Identifies this async operation; stale callbacks after a timeout are ignored.
    pub token: u32,
    /// Host-generated per-port request ID, retained across retries.
    pub request_id: u32,
}

/// Completion of a sent transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Completion {
    /// Port that completed.
    pub port: Port,
    /// Work category.
    pub class: Class,
    /// Whether TX/reply succeeded before its bound expired.
    pub success: bool,
    /// Attempts used, from one to [`MAX_ATTEMPTS`].
    pub attempts: u8,
    /// True when another bounded attempt has already been queued.
    pub retrying: bool,
    /// Host-generated request ID matching the transmitted operation.
    pub request_id: u32,
    /// True when a reply frame arrived, even if parsing/validation failed.
    pub reply_received: bool,
}

#[derive(Clone, Copy)]
struct Job {
    packet: Packet,
    class: Class,
    expects_reply: bool,
    attempt: u8,
    not_before_ms: u32,
    request_id: u32,
}

#[derive(Clone, Copy)]
enum Phase {
    Transmitting { deadline_ms: u32 },
    WaitingReply { deadline_ms: u32 },
}

#[derive(Clone, Copy)]
struct Flight {
    job: Job,
    token: u32,
    phase: Phase,
}

#[derive(Clone, Copy, Default)]
struct PortState {
    motion: Option<Job>,
    diagnostic: Option<Job>,
    configuration: Option<Job>,
    flight: Option<Flight>,
    generation: u32,
    next_request_id: u32,
    ready_at_ms: u32,
    config_allowed_at_ms: u32,
}

impl PortState {
    fn pending_mut(&mut self, class: Class) -> &mut Option<Job> {
        match class {
            Class::Motion => &mut self.motion,
            Class::Diagnostic => &mut self.diagnostic,
            Class::Configuration => &mut self.configuration,
        }
    }

    fn take_ready(&mut self, now_ms: u32, class: Class) -> Option<Job> {
        if self.flight.is_some() || before(now_ms, self.ready_at_ms) {
            return None;
        }
        if class == Class::Configuration && before(now_ms, self.config_allowed_at_ms) {
            return None;
        }
        let slot = self.pending_mut(class);
        match *slot {
            Some(job) if !before(now_ms, job.not_before_ms) => slot.take(),
            _ => None,
        }
    }
}

/// State for both UARTs. Use wrapping-safe deadlines and keep the configured
/// response timeout below half the 32-bit millisecond clock range.
pub struct Scheduler {
    ports: [PortState; 2],
    response_timeout_ms: u32,
    next_port: usize,
}

impl Scheduler {
    /// Creates a scheduler with a finite TX and reply deadline. Zero is rejected.
    pub const fn new(response_timeout_ms: u32) -> Option<Self> {
        if response_timeout_ms == 0 || response_timeout_ms >= 0x8000_0000 {
            return None;
        }
        Some(Self {
            ports: [PortState {
                motion: None,
                diagnostic: None,
                configuration: None,
                flight: None,
                generation: 0,
                next_request_id: 0,
                ready_at_ms: 0,
                config_allowed_at_ms: 0,
            }; 2],
            response_timeout_ms,
            next_port: 0,
        })
    }

    /// Queues the newest goal for a port. It is sent without waiting for a
    /// reply; a newer unsent goal replaces this one. Safing goals use the same
    /// highest-priority path.
    pub fn queue_motion(&mut self, port: Port, packet: Packet) {
        *self.ports[port.index()].pending_mut(Class::Motion) = Some(Job {
            packet,
            class: Class::Motion,
            expects_reply: false,
            attempt: 1,
            not_before_ms: 0,
            request_id: 0,
        });
    }

    /// Queues a safing goal at motion priority, replacing any older unsent
    /// goal for this actuator.
    pub fn queue_safing(&mut self, port: Port, packet: Packet) {
        self.queue_motion(port, packet);
    }

    /// Queues one optional request/reply transaction. Returns false if that
    /// port already has a diagnostic queued; it never blocks motion.
    pub fn queue_diagnostic(&mut self, port: Port, packet: Packet) -> bool {
        self.queue_reply_job(port, packet, Class::Diagnostic)
    }

    /// Queues one configuration request/reply transaction. Returns false if
    /// that port already has configuration work queued. Launches are at least
    /// [`CONFIG_INTERVAL_MS`] apart on that port.
    pub fn queue_configuration(&mut self, port: Port, packet: Packet) -> bool {
        self.queue_reply_job(port, packet, Class::Configuration)
    }

    fn queue_reply_job(&mut self, port: Port, packet: Packet, class: Class) -> bool {
        let slot = self.ports[port.index()].pending_mut(class);
        if slot.is_some() {
            return false;
        }
        *slot = Some(Job {
            packet,
            class,
            expects_reply: true,
            attempt: 1,
            not_before_ms: 0,
            request_id: 0,
        });
        true
    }

    /// Returns at most one ready packet. Call again in the same event-loop
    /// iteration to let the other idle UART start too. The port is reserved
    /// until [`Self::tx_complete`] is called.
    pub fn next_transmission(&mut self, now_ms: u32) -> Option<Transmission> {
        let transaction_timeout_ms = self.response_timeout_ms;
        for class in [Class::Motion, Class::Diagnostic, Class::Configuration] {
            for offset in 0..2 {
                let index = (self.next_port + offset) % 2;
                let state = &mut self.ports[index];
                let Some(mut job) = state.take_ready(now_ms, class) else {
                    continue;
                };
                if job.request_id == 0 {
                    state.next_request_id = state.next_request_id.wrapping_add(1);
                    if state.next_request_id == 0 {
                        state.next_request_id = 1;
                    }
                    job.request_id = state.next_request_id;
                }
                if job.class == Class::Configuration {
                    state.config_allowed_at_ms = now_ms.wrapping_add(CONFIG_INTERVAL_MS);
                }
                state.generation = state.generation.wrapping_add(1);
                let token = state.generation;
                state.flight = Some(Flight {
                    job,
                    token,
                    phase: Phase::Transmitting {
                        deadline_ms: now_ms.wrapping_add(transaction_timeout_ms),
                    },
                });
                self.next_port = (index + 1) % 2;
                return Some(Transmission {
                    port: if index == 0 { Port::A1 } else { Port::A2 },
                    packet: job.packet,
                    class: job.class,
                    attempt: job.attempt,
                    token,
                    request_id: job.request_id,
                });
            }
        }
        None
    }

    /// Reports completion of the nonblocking UART write. On successful
    /// writes, read transactions receive a finite response deadline; goal
    /// writes complete after the required quiet interval.
    pub fn tx_complete(
        &mut self,
        port: Port,
        token: u32,
        now_ms: u32,
        success: bool,
    ) -> Option<Completion> {
        let response_timeout_ms = self.response_timeout_ms;
        let state = &mut self.ports[port.index()];
        let flight = state.flight?;
        if flight.token != token || !matches!(flight.phase, Phase::Transmitting { .. }) {
            return None;
        }
        if !success {
            state.flight = None;
            state.ready_at_ms = now_ms.wrapping_add(WRITE_GAP_MS);
            if flight.job.class == Class::Configuration {
                state.config_allowed_at_ms = now_ms.wrapping_add(CONFIG_INTERVAL_MS);
            }
            return Some(retry_or_finish(state, port, flight.job, now_ms, false, false));
        }
        state.ready_at_ms = now_ms.wrapping_add(WRITE_GAP_MS);
        if flight.job.class == Class::Configuration {
            state.config_allowed_at_ms = now_ms.wrapping_add(CONFIG_INTERVAL_MS);
        }
        if flight.job.expects_reply {
            state.flight = Some(Flight {
                job: flight.job,
                token: flight.token,
                phase: Phase::WaitingReply {
                    deadline_ms: now_ms.wrapping_add(response_timeout_ms),
                },
            });
            None
        } else {
            state.flight = None;
            Some(Completion {
                port,
                class: flight.job.class,
                success: true,
                attempts: flight.job.attempt,
                retrying: false,
                request_id: flight.job.request_id,
                reply_received: false,
            })
        }
    }

    /// Reports a complete, parsed reply or a malformed/invalid reply. An
    /// invalid reply is retried at most once after backoff.
    pub fn reply_received(
        &mut self,
        port: Port,
        token: u32,
        now_ms: u32,
        valid: bool,
    ) -> Option<Completion> {
        let state = &mut self.ports[port.index()];
        let flight = state.flight?;
        if flight.token != token || !matches!(flight.phase, Phase::WaitingReply { .. }) {
            return None;
        }
        state.ready_at_ms = now_ms.wrapping_add(READ_GAP_MS);
        state.flight = None;
        if flight.job.class == Class::Configuration {
            state.config_allowed_at_ms = now_ms.wrapping_add(CONFIG_INTERVAL_MS);
        }
        if valid {
            Some(Completion {
                port,
                class: flight.job.class,
                success: true,
                attempts: flight.job.attempt,
                retrying: false,
                request_id: flight.job.request_id,
                reply_received: true,
            })
        } else {
            Some(retry_or_finish(state, port, flight.job, now_ms, false, true))
        }
    }

    /// Expires overdue replies. Call from the regular event loop; this never
    /// waits. Motion on either port remains eligible while the other retries.
    pub fn tick(&mut self, now_ms: u32) -> [Option<Completion>; 2] {
        let mut completed = [None, None];
        for (index, output) in completed.iter_mut().enumerate() {
            let port = if index == 0 { Port::A1 } else { Port::A2 };
            let state = &mut self.ports[index];
            let Some(flight) = state.flight else {
                continue;
            };
            let (deadline_ms, quiet_ms) = match flight.phase {
                Phase::Transmitting { deadline_ms } => (deadline_ms, WRITE_GAP_MS),
                Phase::WaitingReply { deadline_ms } => (deadline_ms, READ_GAP_MS),
            };
            if !before(now_ms, deadline_ms) {
                state.flight = None;
                state.ready_at_ms = now_ms.wrapping_add(quiet_ms);
                if flight.job.class == Class::Configuration {
                    state.config_allowed_at_ms = now_ms.wrapping_add(CONFIG_INTERVAL_MS);
                }
                *output = Some(retry_or_finish(
                    state,
                    port,
                    flight.job,
                    now_ms,
                    false,
                    false,
                ));
            }
        }
        completed
    }
}

fn retry_or_finish(
    state: &mut PortState,
    port: Port,
    mut job: Job,
    now_ms: u32,
    success: bool,
    reply_received: bool,
) -> Completion {
    let attempts_used = job.attempt;
    let mut retrying = false;
    if !success && job.attempt < MAX_ATTEMPTS {
        job.attempt += 1;
        job.not_before_ms = now_ms.wrapping_add(RETRY_BACKOFF_MS);
        let slot = state.pending_mut(job.class);
        /* A newer goal/operation already queued wins over this retry. */
        if slot.is_none() {
            *slot = Some(job);
            retrying = true;
        }
    }
    Completion {
        port,
        class: job.class,
        success,
        attempts: attempts_used,
        retrying,
        request_id: job.request_id,
        reply_received,
    }
}

fn before(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) >= 0x8000_0000
}
