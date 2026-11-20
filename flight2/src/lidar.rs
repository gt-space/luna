//! Receives TF-Luna UART frames from serial-to-Ethernet bridges.

use std::{
    collections::HashMap,
    io::{self, Read},
    net::{SocketAddr, TcpStream, UdpSocket},
    str::FromStr,
    sync::{Arc, Mutex, Weak},
    thread,
    time::{Duration, Instant},
};

use clap::ValueEnum;
use common::comm::{
    lidar::{Channel, SENSOR_IDS},
    sam::Unit,
    Measurement, MeasurementQuality,
};

const HEADER: u8 = 0x59;
const FRAME_LEN: usize = 9;
const STALE_AFTER: Duration = Duration::from_millis(350);
const READ_TIMEOUT: Duration = Duration::from_millis(200);
const TCP_IDLE_TIMEOUT: Duration = Duration::from_secs(2);
const RETRY_DELAY: Duration = Duration::from_secs(1);

#[derive(Clone, Copy, Debug, ValueEnum)]
pub enum DistanceUnit {
    Cm,
    Mm,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Endpoint {
    Udp(SocketAddr),
    Tcp(SocketAddr),
}

#[derive(Clone, Copy, Debug)]
pub struct LidarConfig {
    pub id: u8,
    pub endpoint: Endpoint,
}

impl FromStr for LidarConfig {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        let (id, endpoint) = value
            .split_once('=')
            .ok_or("expected ID=udp://IP:PORT or ID=tcp://IP:PORT")?;
        let id = id.parse::<u8>().map_err(|_| "LiDAR ID must be 1–4")?;
        if !SENSOR_IDS.contains(&id) {
            return Err("LiDAR ID must be 1–4".into());
        }
        let (transport, addr) = endpoint
            .split_once("://")
            .ok_or("expected udp://IP:PORT or tcp://IP:PORT")?;
        let addr = addr
            .parse::<SocketAddr>()
            .map_err(|_| "expected a numeric IP address and port")?;
        if addr.port() == 0 {
            return Err("LiDAR port must be nonzero".into());
        }
        let endpoint = match transport {
            "udp" => Endpoint::Udp(addr),
            "tcp" if !addr.ip().is_unspecified() => Endpoint::Tcp(addr),
            "tcp" => return Err("TCP requires the bridge's IP address".into()),
            _ => return Err("LiDAR transport must be udp or tcp".into()),
        };
        Ok(Self { id, endpoint })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Frame {
    distance: u16,
    signal_strength: u16,
    temperature: u16,
}

impl Frame {
    fn distance_mm(self, unit: DistanceUnit) -> u32 {
        u32::from(self.distance)
            * match unit {
                DistanceUnit::Cm => 10,
                DistanceUnit::Mm => 1,
            }
    }

    fn quality(self, distance_mm: u32) -> MeasurementQuality {
        match self.signal_strength {
            u16::MAX => MeasurementQuality::SaturatedHigh,
            0..=99 => MeasurementQuality::AtFloor,
            _ if !(200..=8000).contains(&distance_mm) => MeasurementQuality::OutOfRange,
            _ => MeasurementQuality::Ok,
        }
    }
}

#[derive(Default)]
struct FrameParser {
    bytes: [u8; FRAME_LEN],
    len: usize,
}

impl FrameParser {
    fn push(&mut self, byte: u8) -> Option<Frame> {
        if self.len < 2 && byte != HEADER {
            self.len = 0;
            return None;
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        if self.len < FRAME_LEN {
            return None;
        }

        let checksum = self.bytes[..8].iter().copied().fold(0u8, u8::wrapping_add);
        if checksum == self.bytes[8] {
            self.len = 0;
            return Some(Frame {
                distance: u16::from_le_bytes([self.bytes[2], self.bytes[3]]),
                signal_strength: u16::from_le_bytes([self.bytes[4], self.bytes[5]]),
                temperature: u16::from_le_bytes([self.bytes[6], self.bytes[7]]),
            });
        }

        // Retain any overlapping header, including a single trailing 0x59.
        let next_header = self.bytes[1..]
            .windows(2)
            .position(|pair| pair == [HEADER, HEADER])
            .map(|offset| offset + 1)
            .unwrap_or(FRAME_LEN - usize::from(byte == HEADER));
        self.bytes.copy_within(next_header.., 0);
        self.len = FRAME_LEN - next_header;
        None
    }
}

enum Link {
    Udp(UdpSocket),
    Tcp(TcpStream),
}

impl Link {
    fn open(endpoint: Endpoint) -> io::Result<Self> {
        match endpoint {
            Endpoint::Udp(addr) => {
                let socket = UdpSocket::bind(addr)?;
                socket.set_read_timeout(Some(READ_TIMEOUT))?;
                Ok(Self::Udp(socket))
            }
            Endpoint::Tcp(addr) => {
                let stream = TcpStream::connect_timeout(&addr, RETRY_DELAY)?;
                stream.set_read_timeout(Some(READ_TIMEOUT))?;
                Ok(Self::Tcp(stream))
            }
        }
    }

    fn read(&mut self, buffer: &mut [u8]) -> io::Result<(usize, Option<SocketAddr>)> {
        match self {
            Self::Udp(socket) => socket
                .recv_from(buffer)
                .map(|(len, peer)| (len, Some(peer))),
            Self::Tcp(stream) => match stream.read(buffer)? {
                0 => Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "bridge disconnected",
                )),
                len => Ok((len, None)),
            },
        }
    }
}

#[derive(Clone, Copy)]
struct Sample {
    frame: Frame,
    id: u32,
    received_at: Instant,
}

type LatestSample = Mutex<Option<Sample>>;

pub(crate) struct Lidar {
    keys: [String; 3],
    unit: DistanceUnit,
    epoch: Instant,
    latest: Arc<LatestSample>,
    sample: Option<Sample>,
}

impl Lidar {
    pub(crate) fn start(config: LidarConfig, unit: DistanceUnit, epoch: Instant) -> Self {
        let lidar = Self {
            keys: [
                Channel::Distance,
                Channel::SignalStrength,
                Channel::Temperature,
            ]
            .map(|channel| channel.key(config.id)),
            unit,
            epoch,
            latest: Arc::new(Mutex::new(None)),
            sample: None,
        };
        let latest = Arc::downgrade(&lidar.latest);
        if let Err(error) = thread::Builder::new()
            .name(format!("lidar-{}", config.id))
            .spawn(move || read_bridge(config, latest))
        {
            eprintln!("LiDAR {} worker failed to start: {error}", config.id);
        }
        lidar
    }

    pub(crate) fn publish(&mut self, readings: &mut HashMap<String, Measurement>, now: Instant) {
        if let Ok(latest) = self.latest.try_lock() {
            self.sample = *latest;
        }

        let mut fields = [Unit::Millimeters, Unit::Counts, Unit::Celsius].map(|unit| Measurement {
            unit,
            quality: MeasurementQuality::NotSampled,
            ..Default::default()
        });

        if let Some(sample) = self.sample {
            let frame = sample.frame;
            let distance_mm = frame.distance_mm(self.unit);
            let stale = now.saturating_duration_since(sample.received_at) >= STALE_AFTER;
            let sample_time_us = sample.received_at.duration_since(self.epoch).as_micros() as u64;
            let values = [
                (
                    distance_mm as f64,
                    frame.distance,
                    frame.quality(distance_mm),
                ),
                (
                    f64::from(frame.signal_strength),
                    frame.signal_strength,
                    MeasurementQuality::Ok,
                ),
                (
                    f64::from(frame.temperature) / 8.0 - 256.0,
                    frame.temperature,
                    MeasurementQuality::Ok,
                ),
            ];
            for (field, (value, raw, quality)) in fields.iter_mut().zip(values) {
                field.value = value;
                field.raw_count = Some(u32::from(raw));
                field.sample_id = sample.id;
                field.sample_time_us = sample_time_us;
                field.quality = if stale {
                    MeasurementQuality::Stale
                } else {
                    quality
                };
            }
        }

        for (key, field) in self.keys.iter().zip(fields) {
            if let Some(current) = readings.get_mut(key) {
                *current = field;
            } else {
                readings.insert(key.clone(), field);
            }
        }
    }
}

fn read_bridge(config: LidarConfig, latest: Weak<LatestSample>) {
    // Use a buffer large enough for an entire UDP datagram irrespective of the bridge's flush size.
    let mut buffer = vec![0; usize::from(u16::MAX)];
    let mut sample_id = 0;
    while latest.strong_count() > 0 {
        let result = Link::open(config.endpoint)
            .and_then(|link| read_frames(link, &latest, &mut buffer, &mut sample_id));
        if let Err(error) = result {
            eprintln!(
                "LiDAR {} ({:?}): {error}. Retrying.",
                config.id, config.endpoint
            );
            thread::sleep(RETRY_DELAY);
        }
    }
}

fn read_frames(
    mut link: Link,
    latest: &Weak<LatestSample>,
    buffer: &mut [u8],
    sample_id: &mut u32,
) -> io::Result<()> {
    let mut parser = FrameParser::default();
    let mut last_byte = Instant::now();
    let mut last_peer = None;
    while latest.strong_count() > 0 {
        let (len, peer) = match link.read(buffer) {
            Ok((0, _)) => continue,
            Ok(read) => read,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                if matches!(link, Link::Tcp(_)) && last_byte.elapsed() >= TCP_IDLE_TIMEOUT {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "bridge stream idle",
                    ));
                }
                continue;
            }
            Err(error) => return Err(error),
        };
        let received_at = Instant::now();
        if peer != last_peer || received_at.duration_since(last_byte) >= STALE_AFTER {
            parser = FrameParser::default();
        }
        last_peer = peer;
        last_byte = received_at;

        let mut latest_frame = None;
        for &byte in &buffer[..len] {
            if let Some(frame) = parser.push(byte) {
                *sample_id = sample_id.wrapping_add(1).max(1);
                latest_frame = Some(frame);
            }
        }
        if let Some(frame) = latest_frame {
            let Some(latest) = latest.upgrade() else {
                break;
            };
            if let Ok(mut sample) = latest.lock() {
                *sample = Some(Sample {
                    frame,
                    id: *sample_id,
                    received_at,
                });
            };
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::net::TcpListener;

    use super::*;

    const FRAME: [u8; 9] = [0x59, 0x59, 123, 0, 100, 0, 0x18, 0x09, 0xb2];

    #[test]
    fn parser_recovers_after_noise_bad_checksum_and_overlapping_header() {
        let mut corrupt = FRAME;
        corrupt[8] ^= 1;
        let mut parser = FrameParser::default();
        let frames: Vec<_> = [0, HEADER, 0, HEADER]
            .into_iter()
            .chain(corrupt)
            .chain([HEADER])
            .chain(FRAME)
            .chain(FRAME)
            .filter_map(|byte| parser.push(byte))
            .collect();

        assert_eq!(
            frames,
            [Frame {
                distance: 123,
                signal_strength: 100,
                temperature: 2328
            }; 2]
        );
    }

    #[test]
    fn range_and_signal_limits_are_inclusive() {
        let mut frame = Frame {
            distance: 800,
            signal_strength: 100,
            temperature: 0,
        };
        assert_eq!(frame.distance_mm(DistanceUnit::Cm), 8000);
        assert_eq!(frame.distance_mm(DistanceUnit::Mm), 800);
        assert_eq!(frame.quality(200), MeasurementQuality::Ok);
        assert_eq!(frame.quality(8000), MeasurementQuality::Ok);
        for invalid_distance in [0, 199, 8001, u16::MAX.into()] {
            assert_eq!(
                frame.quality(invalid_distance),
                MeasurementQuality::OutOfRange
            );
        }
        frame.signal_strength = 99;
        assert_eq!(frame.quality(1000), MeasurementQuality::AtFloor);
        frame.signal_strength = u16::MAX;
        assert_eq!(frame.quality(1000), MeasurementQuality::SaturatedHigh);
    }

    #[test]
    fn publish_preserves_identity_values_and_sample_time_when_stale() {
        let epoch = Instant::now();
        let mut lidar = Lidar {
            keys: [
                Channel::Distance,
                Channel::SignalStrength,
                Channel::Temperature,
            ]
            .map(|channel| channel.key(3)),
            unit: DistanceUnit::Cm,
            epoch,
            latest: Arc::new(Mutex::new(None)),
            sample: None,
        };
        let mut readings = HashMap::new();
        let distance_key = Channel::Distance.key(3);
        let strength_key = Channel::SignalStrength.key(3);
        let temperature_key = Channel::Temperature.key(3);

        lidar.publish(&mut readings, epoch);
        assert_eq!(
            readings[&distance_key].quality,
            MeasurementQuality::NotSampled
        );
        assert_eq!(readings[&distance_key].raw_count, None);

        let received_at = epoch + Duration::from_millis(50);
        *lidar.latest.lock().unwrap() = Some(Sample {
            frame: Frame {
                distance: 123,
                signal_strength: 99,
                temperature: 2328,
            },
            id: 42,
            received_at,
        });
        lidar.publish(&mut readings, received_at);
        assert_eq!(readings[&distance_key].quality, MeasurementQuality::AtFloor);
        assert_eq!(readings[&distance_key].value, 1230.0);
        assert_eq!(readings[&distance_key].raw_count, Some(123));
        assert_eq!(readings[&strength_key].value, 99.0);
        assert_eq!(readings[&strength_key].quality, MeasurementQuality::Ok);
        assert_eq!(readings[&temperature_key].value, 35.0);

        lidar.publish(&mut readings, received_at + STALE_AFTER);
        for reading in readings.values() {
            assert_eq!(reading.quality, MeasurementQuality::Stale);
            assert_eq!(reading.sample_id, 42);
            assert_eq!(reading.sample_time_us, 50_000);
        }
        assert_eq!(readings[&distance_key].value, 1230.0);
    }

    fn wait_for_sample(latest: &LatestSample, sample_id: u32) -> Sample {
        let deadline = Instant::now() + Duration::from_secs(4);
        loop {
            if let Some(sample) = *latest.lock().unwrap() {
                if sample.id == sample_id {
                    return sample;
                }
            }
            assert!(
                Instant::now() < deadline,
                "timed out waiting for sample {sample_id}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn udp_handles_fragmented_and_coalesced_datagrams_per_source() {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.set_read_timeout(Some(READ_TIMEOUT)).unwrap();
        let address = socket.local_addr().unwrap();
        let first_peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        let second_peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        let latest = Arc::new(Mutex::new(None));
        let weak = Arc::downgrade(&latest);
        let worker =
            thread::spawn(move || read_frames(Link::Udp(socket), &weak, &mut [0; 64], &mut 0));

        first_peer.send_to(&FRAME[..4], address).unwrap();
        second_peer.send_to(&FRAME[4..], address).unwrap();
        first_peer.send_to(&FRAME[..4], address).unwrap();
        first_peer
            .send_to(&[&FRAME[4..], &FRAME].concat(), address)
            .unwrap();
        let sample = wait_for_sample(&latest, 2);
        assert_eq!(sample.frame.distance, 123);

        drop(latest);
        worker.join().unwrap().unwrap();
    }

    #[test]
    fn tcp_reconnects_without_reusing_partial_frames() {
        use std::io::Write;

        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let lidar = Lidar::start(
            LidarConfig {
                id: 1,
                endpoint: Endpoint::Tcp(listener.local_addr().unwrap()),
            },
            DistanceUnit::Cm,
            Instant::now(),
        );
        let accept = || {
            let deadline = Instant::now() + Duration::from_secs(4);
            loop {
                match listener.accept() {
                    Ok((stream, _)) => return stream,
                    Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                        assert!(Instant::now() < deadline, "bridge did not get a connection");
                        thread::sleep(Duration::from_millis(5));
                    }
                    Err(error) => panic!("bridge could not accept connection: {error}"),
                }
            }
        };

        let mut first_connection = accept();
        first_connection
            .write_all(&[&FRAME[..], &FRAME[..4]].concat())
            .unwrap();
        assert_eq!(wait_for_sample(&lidar.latest, 1).frame.distance, 123);
        drop(first_connection);

        let mut second_connection = accept();
        second_connection
            .write_all(&[&FRAME[4..], &FRAME].concat())
            .unwrap();
        assert_eq!(wait_for_sample(&lidar.latest, 2).frame.temperature, 2328);
    }
}
