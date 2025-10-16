# Hardware-in-the-Loop (HIL) Testing Framework

This directory contains a comprehensive testing framework for embedded Rust systems that communicate via UDP. The framework acts as a "fake flight computer" to validate system behavior without modifying the target code.

## Architecture

The HIL testing framework is organized as follows:

```
tests/
├── hil/                      # Main HIL testing framework
│   ├── sam/                 # SAM-specific tests
│   │   ├── test_commands.py      # Valve actuation tests
│   │   ├── test_communication.py # UDP protocol tests
│   │   ├── test_adc.py          # ADC data collection tests
│   │   └── test_integration.py  # Full workflow tests
│   ├── common/              # Shared utilities
│   │   ├── flight_computer.py   # Fake FC UDP client
│   │   ├── mock_hardware.py    # GPIO/SPI mocking
│   │   └── message_types.py     # Python message structures
│   └── requirements.txt     # Python dependencies
├── coverage/                # Coverage reports (gitignored)
├── pytest.ini              # Pytest configuration
└── README.md               # This file
```

## Supported Modes

### Mock Mode (CI/CD)
- **Environment**: `HIL_MODE=mock`
- **Hardware**: Simulated GPIO/SPI interfaces
- **Use Case**: Continuous integration, development testing
- **Benefits**: Fast, no hardware dependencies, repeatable

### Real Mode (Hardware Testing)
- **Environment**: `HIL_MODE=real`
- **Hardware**: Actual GPIO/SPI interfaces
- **Use Case**: Hardware validation, integration testing
- **Benefits**: Real hardware validation, actual sensor data

## Quick Start

### 1. Install Dependencies

```bash
# Install Rust coverage tool
cargo install cargo-llvm-cov

# Install Python dependencies
cd tests/hil
pip install -r requirements.txt
```

### 2. Run Tests

#### Mock Mode (Default)
```bash
cd tests/hil
export HIL_MODE=mock
pytest -v
```

#### Real Mode (Hardware)
```bash
cd tests/hil
export HIL_MODE=real
export SAM_TARGET=192.168.1.100  # Your SAM board IP
pytest -v
```

### 3. Generate Coverage Report

```bash
# Linux/macOS
./tests/coverage.sh

# Windows
tests\coverage.bat
```

## Test Suites

### SAM Tests (`tests/hil/sam/`)

#### Command Tests (`test_commands.py`)
- Valve actuation (channels 1-6)
- Command validation
- Error handling
- Rapid command sequences

#### Communication Tests (`test_communication.py`)
- UDP handshake protocol
- Heartbeat mechanism
- Data transmission
- Protocol robustness

#### ADC Tests (`test_adc.py`)
- Data point structure
- Channel types and units
- Data collection timing
- Sensor simulation

#### Integration Tests (`test_integration.py`)
- Complete workflows
- Abort and recovery
- Performance testing
- Error handling

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `HIL_MODE` | `mock` | Test mode: `mock` or `real` |
| `SAM_TARGET` | `localhost` | SAM board IP address |
| `HIL_DATA_PORT` | `4573` | UDP data port |
| `HIL_COMMAND_PORT` | `8378` | UDP command port |

## Message Protocol

The framework implements the same UDP message protocol used by SAM:

### Data Messages (Port 4573)
- **Identity**: Handshake between FC and SAM
- **FlightHeartbeat**: Keep-alive from FC
- **Sam**: Sensor data from SAM
- **Bms**: Battery data from BMS
- **Ahrs**: Attitude data from AHRS

### Command Messages (Port 8378)
- **ActuateValve**: Control valve power state

## Coverage Reports

The framework generates comprehensive coverage reports:

- **LCOV Format**: `tests/coverage/lcov.info`
- **HTML Report**: `tests/coverage/html/index.html`
- **Coverage Tool**: `cargo llvm-cov`

## Adding New Tests

### For SAM
1. Add test file to `tests/hil/sam/`
2. Follow naming convention: `test_*.py`
3. Use existing fixtures from `conftest.py`

### For Other Subsystems
1. Create new directory: `tests/hil/bms/`, `tests/hil/ahrs/`, etc.
2. Copy structure from `tests/hil/sam/`
3. Adapt message types and test cases

## Example Test

```python
def test_valve_actuation(sam_client, timeout_short):
    """Test valve power on/off."""
    # Power on valve 1
    command = SamControlMessage.actuate_valve(channel=1, powered=True)
    assert sam_client.send_command(command)
    
    # Wait for processing
    time.sleep(timeout_short)
    
    # Power off valve 1
    command = SamControlMessage.actuate_valve(channel=1, powered=False)
    assert sam_client.send_command(command)
```

## Troubleshooting

### Common Issues

1. **Connection Timeout**: Check SAM is running and accessible
2. **No Data Received**: Expected in mock mode, verify real mode setup
3. **Import Errors**: Install Python dependencies with `pip install -r requirements.txt`
4. **Coverage Not Generated**: Ensure `cargo-llvm-cov` is installed

### Debug Mode

```bash
# Enable debug logging
export RUST_LOG=debug
pytest -v -s --tb=long
```

## Future Expansion

The framework is designed to easily support additional subsystems:

- **BMS Testing**: Battery management system validation
- **AHRS Testing**: Attitude and heading reference system
- **Flight Computer Testing**: End-to-end system validation

## Contributing

When adding new tests:

1. Follow existing naming conventions
2. Add appropriate docstrings
3. Use type hints where possible
4. Include both unit and integration tests
5. Update this README if adding new features
