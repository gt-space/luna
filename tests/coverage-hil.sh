#!/bin/bash
# HIL Test Coverage Script for Linux/macOS
# Generates coverage for HIL tests (Python + Rust integration)

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

echo "=== Setting up HIL test environment ==="
cd tests/hil

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

echo "=== Running HIL tests with Python coverage ==="
# Run tests with Python coverage
pytest -v --tb=short --cov=hil --cov-report=html:../coverage/python-html --cov-report=xml:../coverage/python-coverage.xml --cov-report=term

echo "=== Building SAM with coverage instrumentation ==="
cd ../../sam
cargo llvm-cov clean
cargo llvm-cov --no-report --bin sam

echo "=== Running SAM unit tests with coverage ==="
cargo llvm-cov test --no-report

echo "=== Generating Rust coverage report ==="
echo "Generating LCOV report..."
cargo llvm-cov report --lcov --output-path ../tests/coverage/rust-lcov.info

echo "Generating HTML report..."
cargo llvm-cov report --html --output-dir ../tests/coverage/rust-html

echo "=== Coverage reports generated ==="
echo "Python HTML report: tests/coverage/python-html/index.html"
echo "Python XML report: tests/coverage/python-coverage.xml"
echo "Rust HTML report: tests/coverage/rust-html/index.html"
echo "Rust LCOV report: tests/coverage/rust-lcov.info"

echo "=== Coverage script completed ==="
