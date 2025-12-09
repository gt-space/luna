use clap::Parser;
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
    
    // Build column headers by scanning all entries dynamically
    let columns = build_columns_dynamic(&entries);
    println!("Found {} columns", columns.len());
    
    // Write CSV file
    write_csv_dynamic(&output_path, &columns, &entries)?;
    
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
        
        // Validate length to prevent excessive memory allocation
        if len > 100_000_000 {
            return Err(format!("Invalid entry length: {} bytes (too large)", len).into());
        }
        
        // Read the serialized data
        let mut data = vec![0u8; len];
        reader.read_exact(&mut data)?;
        
        // Deserialize
        match postcard::from_bytes::<TimestampedVehicleState>(&data) {
            Ok(entry) => {
                entries.push(entry);
            }
            Err(e) => {
                return Err(format!(
                    "Failed to deserialize entry at position {}: {}. Entry length: {} bytes. \
                    This may indicate a version mismatch or corrupted data.",
                    entries.len(),
                    e,
                    len
                ).into());
            }
        }
    }
    
    Ok(entries)
}

/// Build column headers by dynamically scanning all entries using JSON serialization
fn build_columns_dynamic(entries: &[TimestampedVehicleState]) -> Vec<String> {
    let mut column_set = HashSet::new();
    let mut null_paths = HashSet::new(); // Track paths that are null in some entries
    let mut object_schemas = HashMap::new(); // Track object structures we've seen
    
    // Always include timestamp as first column
    column_set.insert("timestamp".to_string());
    
    // First pass: collect all non-null paths and object schemas
    for entry in entries {
        // Serialize the state to JSON Value
        let json_value = serde_json::to_value(&entry.state)
            .expect("Failed to serialize VehicleState to JSON");
        
        // Recursively extract all paths from the JSON structure
        extract_paths(&json_value, &mut column_set, &mut null_paths, "", &mut object_schemas);
    }
    
    // Add schemas for known Option types that might always be null
    // This ensures we expand fields even when all entries have None
    add_known_option_schemas(&mut object_schemas, &null_paths);
    
    // Second pass: expand null Option fields based on schemas we've seen
    // For example, if "gps" is null but we've seen it as an object with fields,
    // expand it to show all those fields
    let mut expanded_paths = HashSet::new();
    for null_path in &null_paths {
        if let Some(schema) = object_schemas.get(null_path) {
            // This null path corresponds to an object we've seen - expand it
            for field in schema {
                let expanded_path = if null_path.is_empty() {
                    field.clone()
                } else {
                    format!("{}.{}", null_path, field)
                };
                expanded_paths.insert(expanded_path);
            }
        }
    }
    
    // Add expanded paths to column set
    for path in expanded_paths {
        column_set.insert(path);
    }
    
    // Remove null paths that have expanded sub-paths
    // For example, if we have both "gps" (null) and "gps.latitude_deg" (non-null),
    // remove "gps" since it's been expanded
    for path in &null_paths {
        // Check if any column starts with this path followed by a dot
        let prefix = format!("{}.", path);
        if column_set.iter().any(|col| col.starts_with(&prefix)) {
            column_set.remove(path);
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

/// Add schemas for known Option types by creating default instances
/// This ensures we can expand fields even when all entries have None
fn add_known_option_schemas(
    object_schemas: &mut HashMap<String, HashSet<String>>,
    null_paths: &HashSet<String>,
) {
    // Add GPS schema if "gps" appears as a null path but we haven't seen its structure
    // This handles the case where all entries have gps: None
    if null_paths.contains("gps") && !object_schemas.contains_key("gps") {
        // Create a default GpsState to get its schema
        let default_gps = common::comm::GpsState {
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            north_mps: 0.0,
            east_mps: 0.0,
            down_mps: 0.0,
            timestamp_unix_ms: None,
            has_fix: false,
            num_satellites: 0,
        };
        
        // Serialize to JSON to get field names
        if let Ok(json_value) = serde_json::to_value(&default_gps) {
            if let Value::Object(map) = json_value {
                let field_names: HashSet<String> = map.keys().cloned().collect();
                object_schemas.insert("gps".to_string(), field_names);
            }
        }
    }
}

/// Recursively extract all paths from a JSON Value
fn extract_paths(
    value: &Value,
    paths: &mut HashSet<String>,
    null_paths: &mut HashSet<String>,
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
    entries: &[TimestampedVehicleState],
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
            let value = if col == "timestamp" {
                entry.timestamp.to_string()
            } else {
                get_value_from_json(&json_value, col)
            };
            row.push(value);
        }
        
        wtr.write_record(&row)?;
    }
    
    wtr.flush()?;
    Ok(())
}

/// Get a value from JSON Value using a dot-separated path
fn get_value_from_json(value: &Value, path: &str) -> String {
    // Split by '.' but handle array indices like "reco[0].field"
    let parts: Vec<&str> = path.split('.').collect();
    let mut current = value;
    
    for part in parts {
        // Check if this part contains an array index like "reco[0]"
        if part.contains('[') && part.ends_with(']') {
            // Split into key and index: "reco[0]" -> ("reco", 0)
            if let Some(bracket_pos) = part.find('[') {
                let key = &part[..bracket_pos];
                let idx_str = &part[bracket_pos + 1..part.len() - 1];
                
                match current {
                    Value::Object(map) => {
                        if let Some(Value::Array(arr)) = map.get(key) {
                            if let Ok(idx) = idx_str.parse::<usize>() {
                                if let Some(next) = arr.get(idx) {
                                    current = next;
                                } else {
                                    return String::new();
                                }
                            } else {
                                return String::new();
                            }
                        } else {
                            return String::new();
                        }
                    }
                    _ => return String::new(),
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
