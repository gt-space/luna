mod bbone;

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Clone, Parser)]
struct Args {
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
  env_logger::init();
  let args = Args::parse();

  match args.command {
    Command::Bbone { spl, uboot, image } => bbone::flash(&spl, &uboot, &image),
  }
}
