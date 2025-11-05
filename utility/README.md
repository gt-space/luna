# Postcard to CSV Converter

A simple utility to convert flight2 log files (`.postcard` format) into CSV format for easier analysis.

## Usage

```bash
cargo run -- --input path/to/flight_data_YYYYMMDD_HHMMSS.postcard --output output.csv
```

Or build and run:

```bash
cargo build --release
./target/release/postcard-to-csv --input flight_data_20240115_143022.postcard --output output.csv
```

### Arguments

- `--input` / `-i`: Path to the input `.postcard` file (required)
- `--output` / `-o`: Path to the output CSV file (optional, defaults to input filename with `.csv` extension)

## Output Format

The CSV file contains:
- **First column**: `timestamp` - Unix timestamp in seconds with nanosecond precision
- **Remaining columns**: All fields from `VehicleState`, flattened with dot notation

### Column Naming

Nested fields use dot notation with parent prefixes:

- **BMS fields**: `bms.battery_bus.voltage`, `bms.battery_bus.current`, `bms.charger`, etc.
- **AHRS fields**: `ahrs.imu.accelerometer.x`, `ahrs.imu.gyroscope.y`, `ahrs.barometer.temperature`, etc.
- **Valve states**: `valve_states.VALVE_NAME.commanded`, `valve_states.VALVE_NAME.actual`
- **Sensor readings**: `sensor_readings.SENSOR_NAME.value`, `sensor_readings.SENSOR_NAME.unit`
- **Rolling statistics**: `rolling.BOARD_ID.rolling_average_secs`, `rolling.BOARD_ID.delta_time_secs`, etc.
- **Abort stage**: `abort_stage.name`, `abort_stage.abort_condition`, `abort_stage.aborted`

### Example Columns

```
timestamp, bms.battery_bus.voltage, bms.battery_bus.current, bms.charger, ahrs.imu.accelerometer.x, ahrs.imu.accelerometer.y, valve_states.BBV.commanded, valve_states.BBV.actual, sensor_readings.PT1.value, sensor_readings.PT1.unit, ...
```

## Building

```bash
cd utility
cargo build --release
```

The binary will be at `target/release/postcard-to-csv`.

## Notes

- The converter scans all entries in the file to determine all possible column names
- Missing values (e.g., a valve that doesn't exist in all entries) will be empty strings
- Valve state values are printed as debug format (e.g., "Open", "Closed", "Undetermined")
- Unit values are printed as debug format (e.g., "Volts", "Amps", "Psi")
- The `abort_stage.valve_safe_states` field (complex nested HashMap) is not included in the CSV output

