use std::{
    fs::{self, File},
    io::{self, BufReader, Seek, SeekFrom},
    path::PathBuf,
};

use clap::Parser;
use csv::Writer;
use postcard_log::{Columns, FrameReader, Projection};

/// Convert a flight computer log into CSV.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Input .postcard file path
    #[arg(short, long)]
    input: PathBuf,

    /// Output CSV file path (default: input filename with .csv extension)
    #[arg(short, long)]
    output: Option<PathBuf>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let output = args
        .output
        .unwrap_or_else(|| args.input.with_extension("csv"));
    println!("Reading from: {:?}", args.input);
    println!("Writing to: {output:?}");

    let input = File::open(&args.input)?;
    let length = input.metadata()?.len();
    let mut reader = FrameReader::new(BufReader::new(input), length, None).with_legacy_decoding();
    let mut columns = Columns::default();
    let mut count = 0;
    loop {
        match reader.read_record() {
            Ok(Some(record)) => {
                columns.observe(&record);
                count += 1;
            }
            Ok(None) => break,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::InvalidData | io::ErrorKind::UnexpectedEof
                ) =>
            {
                eprintln!(
                    "Warning: {error} at byte {}. Stopping after {count} complete entries.",
                    reader.position()
                );
                break;
            }
            Err(error) => return Err(error.into()),
        }
    }

    println!("Read {count} entries");
    let kind = reader.kind();
    if let Some(kind) = kind {
        println!("Detected file type: {kind:?}");
    }
    let columns = columns.finish();
    println!("Found {} columns", columns.len());

    if fs::canonicalize(&output)
        .is_ok_and(|path| fs::canonicalize(&args.input).is_ok_and(|source| path == source))
    {
        return Err("Choose a CSV output path different from the input log.".into());
    }
    let decoded_end = reader.position();
    let mut input = reader.into_inner();
    input.seek(SeekFrom::Start(0))?;
    let mut reader = FrameReader::new(input, decoded_end, kind).with_legacy_decoding();
    let mut writer = Writer::from_path(output)?;
    let projection = Projection::new(&columns);
    writer.write_record(&columns)?;
    while let Some(entry) = reader.read_record()? {
        writer.write_record(projection.row(&entry))?;
    }
    writer.flush()?;
    println!("Conversion complete!");
    Ok(())
}
