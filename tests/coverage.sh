#!/bin/bash
# HIL Test Coverage Script
# Builds SAM with coverage instrumentation and runs tests

set -e

echo "=== HIL Test Coverage Script ==="

# Check if we're in the right directory
if [ ! -f "sam/Cargo.toml" ]; then
    echo "Error: Must be run from luna repository root"
    exit 1
fi

# Create coverage directory
mkdir -p tests/coverage

echo "=== Installing cargo-llvm-cov ==="
cargo install cargo-llvm-cov

echo "=== Cleaning previous coverage data ==="
cd sam
cargo llvm-cov clean

echo "=== Building SAM with coverage instrumentation ==="
cargo llvm-cov --no-report --bin sam

echo "=== Running HIL tests ==="
cd ../tests/hil

# Set environment for mock mode
export HIL_MODE=mock
export SAM_TARGET=localhost

# Install Python dependencies if needed
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python -m venv venv
fi

source venv/bin/activate 2>/dev/null || source venv/Scripts/activate 2>/dev/null || true

echo "Installing Python dependencies..."
pip install -r requirements.txt

echo "Running pytest..."
pytest -v --tb=short

echo "=== Generating coverage report ==="
cd ../../sam

# Generate coverage report
echo "Generating LCOV report..."
cargo llvm-cov report --lcov --output-path ../tests/coverage/lcov.info

echo "Generating HTML report..."
cargo llvm-cov report --html --output-dir ../tests/coverage/html

echo "=== Coverage report generated ==="
echo "LCOV report: tests/coverage/lcov.info"
echo "HTML report: tests/coverage/html/index.html"

echo "=== Coverage script completed ==="
