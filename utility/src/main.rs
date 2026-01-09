use clap::Parser;
use common::comm::{
    ahrs::{Ahrs, Barometer, Imu, Vector},
    bms::{Bms, Bus},
    CompositeValveState,
};
use csv::Writer;
use serde_json::Value;
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
    let mut column_set = std::collections::HashSet::new();
    
    // Always include timestamp as first column
    column_set.insert("timestamp".to_string());
    
    // First pass: collect all non-null paths and object schemas
    for entry in entries {
        match entry {
            Entry::VehicleState(ts) => {
                add_state_columns(&ts.state, &mut column_set, "");
            }
            Entry::Imu(ts) => {
                add_imu_state_columns(&ts.state, &mut column_set, "");
            }
        }
    }
    
    let mut columns: Vec<String> = column_set.into_iter().collect();
    columns.sort();
    
    // Ensure timestamp is first
    if let Some(pos) = columns.iter().position(|s| s == "timestamp") {
        columns.remove(pos);
        columns.insert(0, "timestamp".to_string());
    }
    
    columns
}

/// Add columns from an IMU state (simpler than VehicleState)
fn add_imu_state_columns(
    imu: &Imu,
    columns: &mut std::collections::HashSet<String>,
    prefix: &str,
) {
    add_imu_columns(imu, columns, prefix);
}

/// Recursively add columns from a VehicleState
fn add_state_columns(
    state: &common::comm::VehicleState,
    columns: &mut std::collections::HashSet<String>,
    prefix: &str,
) {
    // Add BMS columns
    add_bms_columns(&state.bms, columns, &format!("{}bms", prefix));
    
    // Add AHRS columns
    add_ahrs_columns(&state.ahrs, columns, &format!("{}ahrs", prefix));
    
    // Add valve states
    for (valve_name, valve_state) in &state.valve_states {
        let valve_prefix = format!("{}valve_states.{}", prefix, valve_name);
        add_composite_valve_state_columns(valve_state, columns, &valve_prefix);
    }
    
    // Add sensor readings
    for (sensor_name, _measurement) in &state.sensor_readings {
        let sensor_prefix = format!("{}sensor_readings.{}", prefix, sensor_name);
        columns.insert(format!("{}.value", sensor_prefix));
        columns.insert(format!("{}.unit", sensor_prefix));
    }
    
    // Add rolling statistics
    for (board_id, _stats) in &state.rolling {
        let stats_prefix = format!("{}rolling.{}", prefix, board_id);
        columns.insert(format!("{}.rolling_average_secs", stats_prefix));
        columns.insert(format!("{}.delta_time_secs", stats_prefix));
        columns.insert(format!("{}.time_since_last_update", stats_prefix));
    }
    
    // Add abort stage
    let abort_prefix = format!("{}abort_stage", prefix);
    columns.insert(format!("{}.name", abort_prefix));
    columns.insert(format!("{}.abort_condition", abort_prefix));
    columns.insert(format!("{}.aborted", abort_prefix));
    // Note: valve_safe_states is complex (HashMap<String, Vec<ValveAction>>), 
    // we'll skip it for now as it's not typically needed in CSV format
}

/// Add BMS columns
fn add_bms_columns(bms: &Bms, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    add_bus_columns(&bms.battery_bus, columns, &format!("{}.battery_bus", prefix));
    add_bus_columns(&bms.umbilical_bus, columns, &format!("{}.umbilical_bus", prefix));
    add_bus_columns(&bms.sam_power_bus, columns, &format!("{}.sam_power_bus", prefix));
    add_bus_columns(&bms.five_volt_rail, columns, &format!("{}.five_volt_rail", prefix));
    columns.insert(format!("{}.charger", prefix));
    columns.insert(format!("{}.e_stop", prefix));
    columns.insert(format!("{}.rbf_tag", prefix));
}

/// Add Bus columns (voltage and current)
fn add_bus_columns(_bus: &Bus, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.voltage", prefix));
    columns.insert(format!("{}.current", prefix));
}

/// Add AHRS columns
fn add_ahrs_columns(ahrs: &Ahrs, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    add_bus_columns(&ahrs.rail_3v3, columns, &format!("{}.rail_3v3", prefix));
    add_bus_columns(&ahrs.rail_5v, columns, &format!("{}.rail_5v", prefix));
    add_imu_columns(&ahrs.imu, columns, &format!("{}.imu", prefix));
    add_vector_columns(&ahrs.magnetometer, columns, &format!("{}.magnetometer", prefix));
    add_barometer_columns(&ahrs.barometer, columns, &format!("{}.barometer", prefix));
}

/// Add IMU columns
fn add_imu_columns(imu: &Imu, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    let accel_prefix = if prefix.is_empty() {
        "accelerometer".to_string()
    } else {
        format!("{}.accelerometer", prefix)
    };
    let gyro_prefix = if prefix.is_empty() {
        "gyroscope".to_string()
    } else {
        format!("{}.gyroscope", prefix)
    };
    add_vector_columns(&imu.accelerometer, columns, &accel_prefix);
    add_vector_columns(&imu.gyroscope, columns, &gyro_prefix);
}

/// Add Vector columns (x, y, z)
fn add_vector_columns(_vec: &Vector, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.x", prefix));
    columns.insert(format!("{}.y", prefix));
    columns.insert(format!("{}.z", prefix));
}

/// Add Barometer columns
fn add_barometer_columns(_bar: &Barometer, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.temperature", prefix));
    columns.insert(format!("{}.pressure", prefix));
}

/// Add CompositeValveState columns
fn add_composite_valve_state_columns(
    _valve: &CompositeValveState,
    columns: &mut std::collections::HashSet<String>,
    prefix: &str,
    object_schemas: &mut HashMap<String, HashSet<String>>,
) {
    match value {
        Value::Object(map) => {
            // Record this object's schema (all its field names) if we have a prefix
            // This allows us to expand null Option fields later
            if !prefix.is_empty() {
                let field_names: HashSet<String> = map.keys().cloned().collect();
                // Merge with existing schema if any (to handle cases where different entries have different fields)
                object_schemas
                    .entry(prefix.to_string())
                    .and_modify(|existing| {
                        existing.extend(field_names.iter().cloned());
                    })
                    .or_insert_with(|| field_names);
            }
            
            for (key, val) in map {
                let new_prefix = if prefix.is_empty() {
                    key.clone()
                } else {
                    format!("{}.{}", prefix, key)
                };
                extract_paths(val, paths, null_paths, &new_prefix, object_schemas);
            }
        }
        Value::Array(arr) => {
            // For arrays, extract paths from each element using index notation
            // This handles fixed-size arrays like reco: [Option<RecoState>; 3]
            if !arr.is_empty() {
                // Process all elements to discover all possible paths
                for (idx, val) in arr.iter().enumerate() {
                    let idx_prefix = if prefix.is_empty() {
                        format!("[{}]", idx)
                    } else {
                        format!("{}[{}]", prefix, idx)
                    };
                    extract_paths(val, paths, null_paths, &idx_prefix, object_schemas);
                }
            } else {
                // Empty array - still record the path
                paths.insert(prefix.to_string());
            }
        }
        Value::Null => {
            // Track null paths separately - we'll remove them later if they have expanded sub-paths
            null_paths.insert(prefix.to_string());
            // Also add to paths for now, in case it's always null
            paths.insert(prefix.to_string());
        }
        _ => {
            // Primitive value (String, Number, Bool) - this is a leaf node
            paths.insert(prefix.to_string());
        }
    }
}

/// Write CSV file with all entries using dynamic value extraction
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
        let mut row = Vec::with_capacity(columns.len());
        
        // Serialize the state to JSON for dynamic extraction
        let json_value = serde_json::to_value(&entry.state)
            .expect("Failed to serialize VehicleState to JSON");
        
        for col in columns {
            let value = match entry {
                Entry::VehicleState(ts) => get_column_value_vehicle_state(&ts.state, col, ts.timestamp),
                Entry::Imu(ts) => get_column_value_imu(&ts.state, col, ts.timestamp),
            };
            row.push(value);
        }
        
        wtr.write_record(&row)?;
    }
    
    wtr.flush()?;
    Ok(())
}

/// Get the value for a specific column from a VehicleState
fn get_column_value_vehicle_state(state: &common::comm::VehicleState, column: &str, timestamp: f64) -> String {
    if column == "timestamp" {
        return timestamp.to_string();
    }
    
    // Parse the column path (e.g., "bms.battery_bus.voltage")
    let parts: Vec<&str> = column.split('.').collect();
    
    if parts.is_empty() {
        return String::new();
    }
    
    match parts[0] {
        "bms" => get_bms_value(&state.bms, &parts[1..]),
        "ahrs" => get_ahrs_value(&state.ahrs, &parts[1..]),
        "valve_states" => {
            if parts.len() >= 3 {
                let valve_name = parts[1];
                if let Some(valve_state) = state.valve_states.get(valve_name) {
                    get_composite_valve_state_value(valve_state, &parts[2..])
                } else {
                    String::new()
                }
            } else {
                return String::new();
            }
        } else {
            // Regular key access
            match current {
                Value::Object(map) => {
                    if let Some(next) = map.get(part) {
                        current = next;
                    } else {
                        return String::new();
                    }
                }
                Value::Array(arr) => {
                    // If we're at an array and the part is a number, use it as index
                    if let Ok(idx) = part.parse::<usize>() {
                        if let Some(next) = arr.get(idx) {
                            current = next;
                        } else {
                            return String::new();
                        }
                    } else {
                        return String::new();
                    }
                }
                _ => {
                    // Reached a leaf node before finishing the path
                    return String::new();
                }
            }
        }
        _ => String::new(),
    }
}

/// Get the value for a specific column from an Imu
fn get_column_value_imu(imu: &Imu, column: &str, timestamp: f64) -> String {
    if column == "timestamp" {
        return timestamp.to_string();
    }
    
    // Parse the column path (e.g., "accelerometer.x")
    // Filter out empty strings in case column starts with a dot
    let parts: Vec<&str> = column.split('.').filter(|s| !s.is_empty()).collect();
    
    if parts.is_empty() {
        return String::new();
    }
    
    match parts[0] {
        "accelerometer" => get_vector_value(&imu.accelerometer, &parts[1..]),
        "gyroscope" => get_vector_value(&imu.gyroscope, &parts[1..]),
        _ => String::new(),
    }
}

/// Get a value from BMS structure
fn get_bms_value(bms: &Bms, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "battery_bus" => get_bus_value(&bms.battery_bus, &path[1..]),
        "umbilical_bus" => get_bus_value(&bms.umbilical_bus, &path[1..]),
        "sam_power_bus" => get_bus_value(&bms.sam_power_bus, &path[1..]),
        "five_volt_rail" => get_bus_value(&bms.five_volt_rail, &path[1..]),
        "charger" => bms.charger.to_string(),
        "e_stop" => bms.e_stop.to_string(),
        "rbf_tag" => bms.rbf_tag.to_string(),
        _ => String::new(),
    }
}

/// Get a value from Bus structure
fn get_bus_value(bus: &Bus, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "voltage" => bus.voltage.to_string(),
        "current" => bus.current.to_string(),
        _ => String::new(),
    }
}

/// Get a value from AHRS structure
fn get_ahrs_value(ahrs: &Ahrs, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "rail_3v3" => get_bus_value(&ahrs.rail_3v3, &path[1..]),
        "rail_5v" => get_bus_value(&ahrs.rail_5v, &path[1..]),
        "imu" => get_imu_value(&ahrs.imu, &path[1..]),
        "magnetometer" => get_vector_value(&ahrs.magnetometer, &path[1..]),
        "barometer" => get_barometer_value(&ahrs.barometer, &path[1..]),
        _ => String::new(),
    }
}

/// Get a value from IMU structure
fn get_imu_value(imu: &Imu, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "accelerometer" => get_vector_value(&imu.accelerometer, &path[1..]),
        "gyroscope" => get_vector_value(&imu.gyroscope, &path[1..]),
        _ => String::new(),
    }
}

/// Get a value from Vector structure
fn get_vector_value(vec: &Vector, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    // Convert the final value to string
    value_to_string(current)
}

/// Convert a JSON Value to a string representation suitable for CSV
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
        Value::Array(_) => {
            // For arrays, serialize as JSON string
            serde_json::to_string(value).unwrap_or_default()
        }
        Value::Object(_) => {
            // For objects, serialize as JSON string
            serde_json::to_string(value).unwrap_or_default()
        }
    }
}
