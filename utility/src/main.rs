use clap::Parser;
use common::comm::{
    ahrs::{Ahrs, Barometer, Imu, Vector},
    bms::{Bms, Bus},
    CompositeValveState, ValveState,
    sam::Unit,
    Statistics,
};
use csv::Writer;
use postcard::from_bytes;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

/// TimestampedVehicleState structure (must match flight2/src/file_logger.rs)
#[derive(Clone, Debug, serde::Deserialize)]
struct TimestampedVehicleState {
    timestamp: f64,
    state: common::comm::VehicleState,
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
    
    // Build column headers by scanning all entries
    let columns = build_columns(&entries);
    println!("Found {} columns", columns.len());
    
    // Write CSV file
    write_csv(&output_path, &columns, &entries)?;
    
    println!("Conversion complete!");
    Ok(())
}

/// Read all entries from a postcard file
fn read_postcard_file(path: &PathBuf) -> Result<Vec<TimestampedVehicleState>, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut entries = Vec::new();
    
    loop {
        // Read length prefix (8 bytes, u64 little-endian)
        let mut len_bytes = [0u8; 8];
        match reader.read_exact(&mut len_bytes) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break, // EOF
            Err(e) => return Err(e.into()),
        }
        
        let len = u64::from_le_bytes(len_bytes) as usize;
        
        // Read the serialized data
        let mut data = vec![0u8; len];
        reader.read_exact(&mut data)?;
        
        // Deserialize
        let entry: TimestampedVehicleState = from_bytes(&data)?;
        entries.push(entry);
    }
    
    Ok(entries)
}

/// Build column headers by scanning all entries
fn build_columns(entries: &[TimestampedVehicleState]) -> Vec<String> {
    let mut column_set = std::collections::HashSet::new();
    
    // Always include timestamp as first column
    column_set.insert("timestamp".to_string());
    
    for entry in entries {
        add_state_columns(&entry.state, &mut column_set, "");
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
    for (sensor_name, measurement) in &state.sensor_readings {
        let sensor_prefix = format!("{}sensor_readings.{}", prefix, sensor_name);
        columns.insert(format!("{}.value", sensor_prefix));
        columns.insert(format!("{}.unit", sensor_prefix));
    }
    
    // Add rolling statistics
    for (board_id, stats) in &state.rolling {
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
fn add_bus_columns(bus: &Bus, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.voltage", prefix));
    columns.insert(format!("{}.current", prefix));
}

/// Add AHRS columns
fn add_ahrs_columns(ahrs: &Ahrs, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    add_bus_columns(&ahrs.rail_3_3_v, columns, &format!("{}.rail_3_3_v", prefix));
    add_bus_columns(&ahrs.rail_5_v, columns, &format!("{}.rail_5_v", prefix));
    add_imu_columns(&ahrs.imu, columns, &format!("{}.imu", prefix));
    add_vector_columns(&ahrs.magnetometer, columns, &format!("{}.magnetometer", prefix));
    add_barometer_columns(&ahrs.barometer, columns, &format!("{}.barometer", prefix));
}

/// Add IMU columns
fn add_imu_columns(imu: &Imu, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    add_vector_columns(&imu.accelerometer, columns, &format!("{}.accelerometer", prefix));
    add_vector_columns(&imu.gyroscope, columns, &format!("{}.gyroscope", prefix));
}

/// Add Vector columns (x, y, z)
fn add_vector_columns(vec: &Vector, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.x", prefix));
    columns.insert(format!("{}.y", prefix));
    columns.insert(format!("{}.z", prefix));
}

/// Add Barometer columns
fn add_barometer_columns(bar: &Barometer, columns: &mut std::collections::HashSet<String>, prefix: &str) {
    columns.insert(format!("{}.temperature", prefix));
    columns.insert(format!("{}.pressure", prefix));
}

/// Add CompositeValveState columns
fn add_composite_valve_state_columns(
    _valve: &CompositeValveState,
    columns: &mut std::collections::HashSet<String>,
    prefix: &str,
) {
    columns.insert(format!("{}.commanded", prefix));
    columns.insert(format!("{}.actual", prefix));
}

/// Write CSV file with all entries
fn write_csv(
    path: &PathBuf,
    columns: &[String],
    entries: &[TimestampedVehicleState],
) -> Result<(), Box<dyn std::error::Error>> {
    let file = File::create(path)?;
    let mut wtr = Writer::from_writer(file);
    
    // Write header
    wtr.write_record(columns)?;
    
    // Write each row
    for entry in entries {
        let mut row = Vec::with_capacity(columns.len());
        
        for col in columns {
            let value = get_column_value(&entry.state, col, entry.timestamp);
            row.push(value);
        }
        
        wtr.write_record(&row)?;
    }
    
    wtr.flush()?;
    Ok(())
}

/// Get the value for a specific column from a VehicleState
fn get_column_value(state: &common::comm::VehicleState, column: &str, timestamp: f64) -> String {
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
                String::new()
            }
        }
        "sensor_readings" => {
            if parts.len() >= 3 {
                let sensor_name = parts[1];
                if let Some(measurement) = state.sensor_readings.get(sensor_name) {
                    match parts[2] {
                        "value" => measurement.value.to_string(),
                        "unit" => format!("{:?}", measurement.unit),
                        _ => String::new(),
                    }
                } else {
                    String::new()
                }
            } else {
                String::new()
            }
        }
        "rolling" => {
            if parts.len() >= 3 {
                let board_id = parts[1];
                if let Some(stats) = state.rolling.get(board_id) {
                    match parts[2] {
                        "rolling_average_secs" => {
                            stats.rolling_average.as_secs_f64().to_string()
                        }
                        "delta_time_secs" => stats.delta_time.as_secs_f64().to_string(),
                        "time_since_last_update" => stats.time_since_last_update.to_string(),
                        _ => String::new(),
                    }
                } else {
                    String::new()
                }
            } else {
                String::new()
            }
        }
        "abort_stage" => {
            if parts.len() >= 2 {
                match parts[1] {
                    "name" => state.abort_stage.name.clone(),
                    "abort_condition" => state.abort_stage.abort_condition.clone(),
                    "aborted" => state.abort_stage.aborted.to_string(),
                    _ => String::new(),
                }
            } else {
                String::new()
            }
        }
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
        "rail_3_3_v" => get_bus_value(&ahrs.rail_3_3_v, &path[1..]),
        "rail_5_v" => get_bus_value(&ahrs.rail_5_v, &path[1..]),
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
    
    match path[0] {
        "x" => vec.x.to_string(),
        "y" => vec.y.to_string(),
        "z" => vec.z.to_string(),
        _ => String::new(),
    }
}

/// Get a value from Barometer structure
fn get_barometer_value(bar: &Barometer, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "temperature" => bar.temperature.to_string(),
        "pressure" => bar.pressure.to_string(),
        _ => String::new(),
    }
}

/// Get a value from CompositeValveState structure
fn get_composite_valve_state_value(valve: &CompositeValveState, path: &[&str]) -> String {
    if path.is_empty() {
        return String::new();
    }
    
    match path[0] {
        "commanded" => format!("{:?}", valve.commanded),
        "actual" => format!("{:?}", valve.actual),
        _ => String::new(),
    }
}

