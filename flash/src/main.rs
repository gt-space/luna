mod bbone;

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Clone, Parser)]
struct Args {
  #[arg(long, short, global = true)]
  verbose: bool,

  #[command(subcommand)]
  command: Command,
}

#[derive(Clone, Subcommand)]
enum Command {
  Bbone {
    #[arg(long)]
    spl: PathBuf,

    #[arg(long)]
    uboot: PathBuf,

    #[arg(long, short)]
    image: PathBuf,
  },
}

fn main() {
  let args = Args::parse();

  env_logger::Builder::new()
    .filter_level(if args.verbose {
      log::LevelFilter::Debug
    } else {
      log::LevelFilter::Info
    })
    .parse_default_env()
    .init();

  match args.command {
    Command::Bbone { spl, uboot, image } => bbone::flash(&spl, &uboot, &image),
  }
}
