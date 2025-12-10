use clap::Parser as ArgParser;
use rand::{Rng, SeedableRng, rngs::StdRng};
use socket2::{Domain, Protocol, Socket, Type as SocketType};
use std::{io, net::SocketAddr, thread, time::Duration};

#[derive(ArgParser)]
struct Args {
  #[arg(short = 'n', long, default_value_t = 0)]
  count: u64,

  #[arg(short = 't', long, default_value_t = 1000)]
  delay: u64,

  #[arg(long, default_value = "192.168.1.10:7201")]
  dest: SocketAddr,

  #[arg(long, default_value_t = 10)]
  dscp: u8,

  #[arg(long, default_value_t = 0)]
  seed: u64,

  #[arg(short, long, default_value_t = 100)]
  size: usize,
}

fn main() -> io::Result<()> {
  let args = Args::parse();
  let infinite = args.count == 0;

  let socket = Socket::new(Domain::IPV4, SocketType::DGRAM, Some(Protocol::UDP))?;
  socket.set_tos_v4((args.dscp as u32) << 2)?;

  let mut rng = StdRng::seed_from_u64(args.seed);
  let mut buffer = vec![0_u8; args.size];
  let mut sent = 0;

  loop {
    // Sent random packet.
    rng.fill(buffer.as_mut_slice());
    socket.send_to(&buffer, &args.dest.into())?;

    sent += 1;
    println!("Sent packet {sent}");

    if !infinite && sent >= args.count {
      break;
    }

    if args.delay > 0 {
      thread::sleep(Duration::from_millis(args.delay));
    }
  }

  println!("Done. Sent {sent} packets.");
  Ok(())
}
