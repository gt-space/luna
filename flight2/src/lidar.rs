//! Receives the vehicle's TF-Luna UART frames over TCP.

use std::{
    collections::HashMap,
    io::{self, Read},
    net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpStream},
    sync::{Arc, Mutex, Weak},
    thread,
    time::{Duration, Instant},
};

use common::comm::{lidar::Channel, sam::Unit, Measurement, MeasurementQuality};

const HEADER: u8 = 0x59;
const FRAME_LEN: usize = 9;
const STALE_AFTER: Duration = Duration::from_millis(350);
const READ_TIMEOUT: Duration = Duration::from_millis(200);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(1);
const TCP_IDLE_TIMEOUT: Duration = Duration::from_secs(2);
const RETRY_DELAY: Duration = Duration::from_secs(1);
const ADAPTER_PORT: u16 = 2000;

pub(crate) const ADAPTERS: [(u8, SocketAddr); 2] = [
    (
        1,
        SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::new(192, 168, 1, 200),
            ADAPTER_PORT,
        )),
    ),
    (
        2,
        SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::new(192, 168, 1, 201),
            ADAPTER_PORT,
        )),
    ),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Frame {
    distance_cm: u16,
    signal_strength: u16,
    temperature: u16,
}

impl Frame {
    fn distance_mm(self) -> u32 {
        u32::from(self.distance_cm) * 10
    }

    fn quality(self) -> MeasurementQuality {
        match self.signal_strength {
            u16::MAX => MeasurementQuality::SaturatedHigh,
            0..=99 => MeasurementQuality::AtFloor,
            _ if !(20..=800).contains(&self.distance_cm) => MeasurementQuality::OutOfRange,
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
                distance_cm: u16::from_le_bytes([self.bytes[2], self.bytes[3]]),
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

#[derive(Clone, Copy)]
struct Sample {
    frame: Frame,
    id: u32,
    received_at: Instant,
}

type LatestSample = Mutex<Option<Sample>>;

pub(crate) struct Lidar {
    keys: [String; 3],
    epoch: Instant,
    latest: Arc<LatestSample>,
    sample: Option<Sample>,
}

impl Lidar {
    pub(crate) fn start_pair() -> [Self; 2] {
        let epoch = Instant::now();
        ADAPTERS.map(|(id, endpoint)| Self::start(id, endpoint, epoch))
    }

    fn start(id: u8, endpoint: SocketAddr, epoch: Instant) -> Self {
        let lidar = Self {
            keys: [
                Channel::Distance,
                Channel::SignalStrength,
                Channel::Temperature,
            ]
            .map(|channel| channel.key(id)),
            epoch,
            latest: Arc::new(Mutex::new(None)),
            sample: None,
        };
        let latest = Arc::downgrade(&lidar.latest);
        if let Err(error) = thread::Builder::new()
            .name(format!("lidar-{id}"))
            .spawn(move || read_bridge(id, endpoint, latest))
        {
            eprintln!("LiDAR {id} worker failed to start: {error}");
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
            let stale = now.saturating_duration_since(sample.received_at) >= STALE_AFTER;
            let sample_time_us = sample.received_at.duration_since(self.epoch).as_micros() as u64;
            let values = [
                (
                    f64::from(frame.distance_mm()),
                    frame.distance_cm,
                    frame.quality(),
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

fn read_bridge(id: u8, endpoint: SocketAddr, latest: Weak<LatestSample>) {
    let mut sample_id = 0;
    while latest.strong_count() > 0 {
        let result = TcpStream::connect_timeout(&endpoint, CONNECT_TIMEOUT)
            .and_then(|stream| read_frames(stream, &latest, &mut sample_id));
        if let Err(error) = result {
            eprintln!("LiDAR {id} ({endpoint}): {error}. Retrying.");
            thread::sleep(RETRY_DELAY);
        }
    }
}

fn read_frames(
    mut stream: TcpStream,
    latest: &Weak<LatestSample>,
    sample_id: &mut u32,
) -> io::Result<()> {
    stream.set_read_timeout(Some(READ_TIMEOUT))?;
    let mut buffer = [0; 1024];
    let mut parser = FrameParser::default();
    let mut last_byte = Instant::now();
    while latest.strong_count() > 0 {
        let len = match stream.read(&mut buffer) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "bridge disconnected",
                ));
            }
            Ok(read) => read,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                if last_byte.elapsed() >= TCP_IDLE_TIMEOUT {
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
    use std::{io::Write, net::TcpListener};

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
                distance_cm: 123,
                signal_strength: 100,
                temperature: 2328
            }; 2]
        );
    }

    #[test]
    fn range_and_signal_limits_are_inclusive() {
        let mut frame = Frame {
            distance_cm: 800,
            signal_strength: 100,
            temperature: 0,
        };
        assert_eq!(frame.distance_mm(), 8000);
        assert_eq!(frame.quality(), MeasurementQuality::Ok);
        frame.distance_cm = 20;
        assert_eq!(frame.distance_mm(), 200);
        assert_eq!(frame.quality(), MeasurementQuality::Ok);
        for invalid_distance in [0, 19, 801, u16::MAX] {
            frame.distance_cm = invalid_distance;
            assert_eq!(frame.quality(), MeasurementQuality::OutOfRange);
        }
        frame.distance_cm = 100;
        frame.signal_strength = 99;
        assert_eq!(frame.quality(), MeasurementQuality::AtFloor);
        frame.signal_strength = u16::MAX;
        assert_eq!(frame.quality(), MeasurementQuality::SaturatedHigh);
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
            .map(|channel| channel.key(2)),
            epoch,
            latest: Arc::new(Mutex::new(None)),
            sample: None,
        };
        let mut readings = HashMap::new();
        let distance_key = Channel::Distance.key(2);
        let strength_key = Channel::SignalStrength.key(2);
        let temperature_key = Channel::Temperature.key(2);

        lidar.publish(&mut readings, epoch);
        assert_eq!(
            readings[&distance_key].quality,
            MeasurementQuality::NotSampled
        );
        assert_eq!(readings[&distance_key].raw_count, None);

        let received_at = epoch + Duration::from_millis(50);
        *lidar.latest.lock().unwrap() = Some(Sample {
            frame: Frame {
                distance_cm: 123,
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

    fn accept(listener: &TcpListener) -> TcpStream {
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
    }

    #[test]
    fn vehicle_adapter_addresses_match_sensor_ids() {
        assert_eq!(
            ADAPTERS,
            [
                (1, "192.168.1.200:2000".parse().unwrap()),
                (2, "192.168.1.201:2000".parse().unwrap()),
            ]
        );
    }

    #[test]
    fn each_tcp_stream_publishes_its_own_samples() {
        let listeners = [(); 2].map(|_| {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            listener.set_nonblocking(true).unwrap();
            listener
        });
        let epoch = Instant::now();
        let mut first = Lidar::start(1, listeners[0].local_addr().unwrap(), epoch);
        let mut second = Lidar::start(2, listeners[1].local_addr().unwrap(), epoch);
        let mut first_bridge = accept(&listeners[0]);
        let mut second_bridge = accept(&listeners[1]);

        first_bridge.write_all(&FRAME[..4]).unwrap();
        second_bridge.write_all(&FRAME).unwrap();
        wait_for_sample(&second.latest, 1);
        assert!(first.latest.lock().unwrap().is_none());

        first_bridge
            .write_all(&[&FRAME[4..], &FRAME].concat())
            .unwrap();
        wait_for_sample(&first.latest, 2);
        let mut readings = HashMap::new();
        first.publish(&mut readings, Instant::now());
        second.publish(&mut readings, Instant::now());
        assert_eq!(readings.len(), 6);
        assert_eq!(readings[&Channel::Distance.key(1)].sample_id, 2);
        assert_eq!(readings[&Channel::Distance.key(2)].sample_id, 1);
    }

    #[test]
    fn tcp_reconnects_without_reusing_partial_frames() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let lidar = Lidar::start(1, listener.local_addr().unwrap(), Instant::now());

        let mut first_connection = accept(&listener);
        first_connection
            .write_all(&[&FRAME[..], &FRAME[..4]].concat())
            .unwrap();
        assert_eq!(wait_for_sample(&lidar.latest, 1).frame.distance_cm, 123);
        drop(first_connection);

        let mut second_connection = accept(&listener);
        second_connection
            .write_all(&[&FRAME[4..], &FRAME].concat())
            .unwrap();
        assert_eq!(wait_for_sample(&lidar.latest, 2).frame.temperature, 2328);
    }
}
