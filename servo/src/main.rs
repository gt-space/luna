use clap::{builder::PossibleValuesParser, Arg, ArgAction, Command};
use jeflog::fail;
use servo::tool;
use std::{
  env,
  fs,
  path::{Path, PathBuf},
  process,
};

fn main() -> anyhow::Result<()> {
  #[cfg(target_family = "windows")]
  let home_path = &env::var("USERPROFILE")?;

  #[cfg(target_family = "unix")]
  let home_path = &env::var("HOME")?;

  let servo_dir = Path::new(home_path).join(".servo");

  if !servo_dir.is_dir() {
    fs::create_dir(&servo_dir).unwrap();
  }

  let matches = Command::new("servo")
    .about("Servo command line tool")
    .subcommand_required(true)
    .subcommand(
      Command::new("clean")
        .about("Cleans the Servo directory and database.")
    )
    .subcommand(
      Command::new("deploy")
        .about("Deploys YJSP software to all available computers on the network.")
        .arg(
          Arg::new("prepare")
            .long("prepare")
            .required(false)
            .num_args(0),
        )
        .arg(
          Arg::new("offline")
            .long("offline")
            .required(false)
            .num_args(0),
        )
        .arg(Arg::new("to").long("to").short('t').required(false))
        .arg(Arg::new("path").long("path").short('p').required(false)),
    )
    .subcommand(
      Command::new("emulate")
        .about("Emulates a particular subsystem of the YJSP software stack.")
        .arg(
          Arg::new("component")
            .required(true)
            .ignore_case(true)
            .value_parser(PossibleValuesParser::new(["flight", "sam"])),
        )
        .arg(
          Arg::new("frequency")
            .required(false)
            .default_value("100.0")
            .short('f')
            .value_parser(clap::value_parser!(f64)),
        )
        .arg(
          Arg::new("duration")
            .required(false)
            .short('t')
            .value_parser(clap::value_parser!(f64)),
        ),
    )
    .subcommand(
      Command::new("export")
        .about("Exports vehicle state data from a specified timestamp to a specified timestamp.")
        .arg(Arg::new("output_path").required(true).short('o'))
        .arg(
          Arg::new("from")
            .required(false)
            .long("from")
            .short('f')
        )
        .arg(
          Arg::new("to")
            .required(false)
            .long("to")
            .short('t')
        )
        .arg(
          Arg::new("all")
            .short('a')
            .action(ArgAction::SetTrue)
        ),
    )
    .subcommand(
      Command::new("locate")
        .about("Locates the IP addresses of known hostnames on the network.")
        .arg(
          Arg::new("subsystem")
            .required(false)
            .value_parser(
              PossibleValuesParser::new(["gui", "servo", "flight", "sam"])
            ),
        ),
    )
    .subcommand(
      Command::new("run")
        .about("Sends a Python sequence to be run on the flight computer.")
        .arg(Arg::new("path").required(true)),
    )
    .subcommand(
      Command::new("serve")
        .about("Starts the servo server.")
        .arg(
          Arg::new("volatile")
            .long("volatile")
            .action(ArgAction::SetTrue),
        )
        .arg(
          Arg::new("quiet")
            .long("quiet")
            .short('q')
            .action(ArgAction::SetTrue),
        ),
    )
    .subcommand(
      Command::new("sql")
        .about("Executes a SQL statement on the control server database and displays the result.")
        .arg(Arg::new("raw_sql").required(true)),
    )
    .subcommand(
      Command::new("upload")
        .about("Uploads a Python sequence to the control server to be stored for future use.")
        .arg(
          Arg::new("sequence_path")
            .value_parser(clap::value_parser!(PathBuf))
            .required(true),
        ),
    )
    .get_matches();

  match matches.subcommand() {
    Some(("clean", _)) | Some(("nuke", _))=> tool::clean(&servo_dir)?,
    Some(("deploy", args)) => tool::deploy(args),
    Some(("emulate", args)) => tool::emulate(args)?,
    Some(("export", args)) => {
      tool::export(
        args.get_one::<String>("from").cloned(),
        args.get_one::<String>("to").cloned(),
        args.get_one::<String>("output_path").unwrap(),
        args.get_one::<bool>("all").unwrap(),
      )?;
    }
    Some(("locate", args)) => tool::locate(args)?,
    Some(("purge-data", args)) => tool::purge_data()?,
    Some(("run", args)) => tool::run(args.get_one::<String>("path").unwrap())?,
    Some(("serve", args)) => tool::serve(&servo_dir, args)?,
    Some(("sql", args)) => {
      tool::sql(args.get_one::<String>("raw_sql").unwrap())?
    }
    Some(("upload", args)) => {
      tool::upload(args.get_one::<PathBuf>("sequence_path").unwrap())?
    }
    _ => {
      fail!("Invalid command. Please check the command you entered.");
      process::exit(1);
    }
  };

  Ok(())
}
