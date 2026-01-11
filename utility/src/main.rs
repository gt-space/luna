use clap::Parser;
use common::comm::{
    ahrs::{Ahrs, Barometer, Imu, Vector},
    bms::{Bms, Bus},
    CompositeValveState,
};
use csv::Writer;
use serde_json::Value;
use postcard::from_bytes;
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

/// TimestampedVehicleState structure (must match flight2/src/file_logger.rs)
#[derive(Clone, Debug, serde::Deserialize)]
struct TimestampedVehicleState {
    timestamp: f64,
    state: common::comm::VehicleState,
}

/// TimestampedImu structure (must match ahrs/src/file_logger.rs)
#[derive(Clone, Debug, serde::Deserialize)]
struct TimestampedImu {
    timestamp: f64,
    state: Imu,
}

/// Enum to represent either type of entry
#[derive(Clone, Debug)]
enum Entry {
    VehicleState(TimestampedVehicleState),
    Imu(TimestampedImu),
}

/// Command-line arguments
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
    
    // Determine output path
    let output_path = args.output.unwrap_or_else(|| {
        let mut path = args.input.clone();
        path.set_extension("csv");
        path
    });
    
    println!("Reading from: {:?}", args.input);
    println!("Writing to: {:?}", output_path);
    
    // Read and parse the postcard file
    let entries = read_postcard_file(&args.input)?;
    println!("Read {} entries", entries.len());
    
    // Determine file type from first entry
    let file_type = entries.first().map(|e| match e {
        Entry::VehicleState(_) => "VehicleState",
        Entry::Imu(_) => "Imu",
    });
    if let Some(ft) = file_type {
        println!("Detected file type: {}", ft);
    }
    
    // Build column headers by scanning all entries
    let columns = build_columns(&entries);
    println!("Found {} columns", columns.len());
    
    // Write CSV file
    write_csv_dynamic(&output_path, &columns, &entries)?;
    
    println!("Conversion complete!");
    Ok(())
}

/// Read all entries from a postcard file
/// Tries to deserialize as VehicleState first, then falls back to Imu
fn read_postcard_file(path: &PathBuf) -> Result<Vec<Entry>, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut entries = Vec::new();
    
    // Try to determine file type from first entry
    let mut file_type: Option<bool> = None; // None = unknown, true = VehicleState, false = Imu
    
    loop {
        // Read length prefix (8 bytes, u64 little-endian)
        let mut len_bytes = [0u8; 8];
        match reader.read_exact(&mut len_bytes) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break, // EOF
            Err(e) => return Err(e.into()),
        }
        
        let len = u64::from_le_bytes(len_bytes) as usize;

        // Treat obviously invalid lengths specially so we can still recover earlier data.
        // A zero-length entry should never be produced by our logger (postcard-encoded
        // structs are always at least 1 byte), so this almost certainly indicates a
        // truncated or otherwise corrupted tail of the file.
        if len == 0 {
            eprintln!(
                "Warning: encountered zero-length entry at position {}. \
                 Treating this as a corrupted/truncated tail and stopping at {} complete entries.",
                entries.len(),
                entries.len()
            );
            break;
        }
        
        // Validate length to prevent excessive memory allocation
        if len > 100_000_000 {
            return Err(format!("Invalid entry length: {} bytes (too large)", len).into());
        }
        
        // Read the serialized data
        let mut data = vec![0u8; len];
        match reader.read_exact(&mut data) {
            Ok(_) => {}
            // If we hit EOF in the middle of an entry, treat it as a truncated
            // final record: stop reading and return everything we've got so far.
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                eprintln!(
                    "Warning: encountered truncated final entry (expected {} bytes, got fewer). \
                    Stopping at {} complete entries.",
                    len,
                    entries.len()
                );
                break;
            }
            Err(e) => return Err(e.into()),
        }
        
        // Try to deserialize based on detected file type, or try both if unknown
        let entry = match file_type {
            Some(true) => {
                // Known to be VehicleState
                let entry: TimestampedVehicleState = from_bytes(&data)?;
                Entry::VehicleState(entry)
            }
            Some(false) => {
                // Known to be Imu
                let entry: TimestampedImu = from_bytes(&data)?;
                Entry::Imu(entry)
            }
            None => {
                // Unknown type - try VehicleState first
                match from_bytes::<TimestampedVehicleState>(&data) {
                    Ok(entry) => {
                        file_type = Some(true);
                        Entry::VehicleState(entry)
                    }
                    Err(_) => {
                        // Try Imu
                        let entry: TimestampedImu = from_bytes(&data)?;
                        file_type = Some(false);
                        Entry::Imu(entry)
                    }
                }
            }
        };
        
        entries.push(entry);
    }
    
    Ok(entries)
}

/// Build column headers by scanning all entries
fn build_columns(entries: &[Entry]) -> Vec<String> {
    use std::collections::HashSet;

    let mut paths: HashSet<String> = HashSet::new();

    // Always include timestamp as first column
    paths.insert("timestamp".to_string());

    // Collect all JSON paths from every entry's state
    for entry in entries {
        let value = match entry {
            Entry::VehicleState(ts) => serde_json::to_value(&ts.state)
                .expect("Failed to serialize VehicleState to JSON"),
            Entry::Imu(ts) => serde_json::to_value(&ts.state)
                .expect("Failed to serialize Imu to JSON"),
        };

        extract_paths(&value, &mut paths, "");
    }

    let mut columns: Vec<String> = paths.into_iter().collect();
    columns.sort();

    // Ensure timestamp is first
    if let Some(pos) = columns.iter().position(|s| s == "timestamp") {
        columns.remove(pos);
        columns.insert(0, "timestamp".to_string());
    }

    columns
}

/// Recursively collect all JSON paths in a value.
/// Paths use dot notation for objects and `[idx]` for array indices.
fn extract_paths(value: &Value, paths: &mut std::collections::HashSet<String>, prefix: &str) {
    match value {
        Value::Object(map) => {
            for (key, val) in map {
                let new_prefix = if prefix.is_empty() {
                    key.clone()
                } else {
                    format!("{}.{}", prefix, key)
                };
                extract_paths(val, paths, &new_prefix);
            }
        }
        Value::Array(arr) => {
            if arr.is_empty() {
                if !prefix.is_empty() {
                    paths.insert(prefix.to_string());
                }
            } else {
                for (idx, val) in arr.iter().enumerate() {
                    let idx_prefix = if prefix.is_empty() {
                        format!("[{}]", idx)
                    } else {
                        format!("{}[{}]", prefix, idx)
                    };
                    extract_paths(val, paths, &idx_prefix);
                }
            }
        }
        Value::Null | Value::Bool(_) | Value::Number(_) | Value::String(_) => {
            if !prefix.is_empty() {
                paths.insert(prefix.to_string());
            }
        }
    }
}

/// Write CSV file with all entries using dynamic JSON path extraction
fn write_csv_dynamic(
    path: &PathBuf,
    columns: &[String],
    entries: &[Entry],
) -> Result<(), Box<dyn std::error::Error>> {
    let file = File::create(path)?;
    let mut wtr = Writer::from_writer(file);

    // Write header
    wtr.write_record(columns)?;

    // Write each row
    for entry in entries {
        let (timestamp, json_value) = match entry {
            Entry::VehicleState(ts) => (
                ts.timestamp,
                serde_json::to_value(&ts.state)
                    .expect("Failed to serialize VehicleState to JSON"),
            ),
            Entry::Imu(ts) => (
                ts.timestamp,
                serde_json::to_value(&ts.state)
                    .expect("Failed to serialize Imu to JSON"),
            ),
        };

        let mut row = Vec::with_capacity(columns.len());

        for col in columns {
            if col == "timestamp" {
                row.push(timestamp.to_string());
            } else if let Some(value) = get_value_at_path(&json_value, col) {
                row.push(value_to_string(value));
            } else {
                row.push(String::new());
            }
        }

        wtr.write_record(&row)?;
    }

    wtr.flush()?;
    Ok(())
}

/// Follow a dot/array path like `bms.battery_bus.voltage` or `reco[0].altitude` in a JSON value.
fn get_value_at_path<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    let mut current = value;
    let chars: Vec<char> = path.chars().collect();
    let mut i = 0;

    while i < chars.len() {
        match chars[i] {
            '.' => {
                // Skip redundant dots
                i += 1;
            }
            '[' => {
                // Parse array index
                i += 1;
                let start = i;
                while i < chars.len() && chars[i].is_ascii_digit() {
                    i += 1;
                }
                if i == start || i >= chars.len() || chars[i] != ']' {
                    return None;
                }
                let idx: usize = chars[start..i].iter().collect::<String>().parse().ok()?;
                i += 1; // skip ']'
                if let Value::Array(arr) = current {
                    current = arr.get(idx)?;
                } else {
                    return None;
                }
            }
            _ => {
                // Parse object key until '.' or '['
                let start = i;
                while i < chars.len() && chars[i] != '.' && chars[i] != '[' {
                    i += 1;
                }
                let key: String = chars[start..i].iter().collect();
                if key.is_empty() {
                    continue;
                }
                if let Value::Object(map) = current {
                    current = map.get(&key)?;
                } else {
                    return None;
                }
            }
        }
    }

    Some(current)
}

/// Convert a JSON `Value` to a string representation suitable for CSV
fn value_to_string(value: &Value) -> String {
    match value {
        Value::Null => String::new(),
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => {
            // Try to preserve precision for floats
            if let Some(f) = n.as_f64() {
                f.to_string()
            } else if let Some(i) = n.as_i64() {
                i.to_string()
            } else if let Some(u) = n.as_u64() {
                u.to_string()
            } else {
                n.to_string()
            }
        }
        Value::String(s) => s.clone(),
        Value::Array(_) | Value::Object(_) => {
            // For arrays and objects, serialize as JSON string
            serde_json::to_string(value).unwrap_or_default()
        }
    }
}
