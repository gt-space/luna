//! The path that carries simulated sensor samples into the flight computer's
//! main loop, only in a build with the `sil` feature.
//!
//! A flight build has no such path: nothing in it listens for sensor data
//! from the network. With `sil`, `--sil-feed <PORT>` opens a UDP socket, and
//! the SIL bridge (`sil_bridge.rs`) sends [`SilSample`]s to it that the main
//! loop hands to the navigation filter and the vehicle state exactly where it
//! hands the real drivers' samples.

use std::{
    io,
    net::{SocketAddr, UdpSocket},
};

use crate::sil_sensors::Stamped;

/// Largest sample on the wire, bytes. A GPS solution is the biggest.
const MAX_DATAGRAM: usize = 256;

/// The receiving end, in the flight computer.
#[derive(Debug)]
pub struct SilFeed {
    socket: UdpSocket,
}

impl SilFeed {
    /// Listens on `port`, on every interface of this host.
    pub fn bind(port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", port))?;

        socket.set_nonblocking(true)?;

        Ok(SilFeed { socket })
    }

    /// Every sample that has arrived, oldest first. Datagrams that do not
    /// decode are dropped.
    pub fn poll(&self) -> Vec<Stamped> {
        let mut samples = Vec::new();
        let mut buffer = [0u8; MAX_DATAGRAM];

        while let Ok(size) = self.socket.recv(&mut buffer) {
            if let Some(sample) = Stamped::decode(&buffer[..size]) {
                samples.push(sample);
            }
        }

        samples
    }
}

/// The sending end, in the bridge.
#[derive(Debug)]
pub struct SilFeedSender {
    socket: UdpSocket,
    target: SocketAddr,
}

impl SilFeedSender {
    pub fn new(target: SocketAddr) -> io::Result<Self> {
        Ok(SilFeedSender {
            socket: UdpSocket::bind(("0.0.0.0", 0))?,
            target,
        })
    }

    /// Sends one sample. A send that fails is not an error worth stopping
    /// for: the receiver may not be up yet.
    pub fn send(&self, sample: &Stamped) {
        let _ = self.socket.send_to(&sample.encode(), self.target);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::sil_sensors::SilSample;

    #[test]
    fn samples_arrive_in_order_and_junk_is_dropped() {
        let feed = SilFeed::bind(0).unwrap();
        let port = feed.socket.local_addr().unwrap().port();
        let sender = SilFeedSender::new(SocketAddr::from(([127, 0, 0, 1], port))).unwrap();

        let stamp = |t_s, sample| Stamped { t_s, sample };
        let one = stamp(
            1.0,
            SilSample::Mag {
                gauss: [1.0, 2.0, 3.0],
            },
        );
        let two = stamp(
            2.0,
            SilSample::Baro {
                pressure_pa: 97_000.0,
                temperature_c: 20.0,
            },
        );

        sender.send(&one);
        sender
            .socket
            .send_to(&[0xFF, 0xFF, 0xFF, 0xFF], sender.target)
            .unwrap();
        sender.send(&two);

        std::thread::sleep(Duration::from_millis(50));

        assert_eq!(feed.poll(), vec![one, two]);
        assert!(feed.poll().is_empty());
    }
}
