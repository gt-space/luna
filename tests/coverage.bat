@echo off
REM HIL Test Coverage Script for Windows
REM Builds SAM with coverage instrumentation and runs tests

echo === HIL Test Coverage Script ===

REM Check if we're in the right directory
if not exist "sam\Cargo.toml" (
    echo Error: Must be run from luna repository root
    exit /b 1
)

REM Create coverage directory
if not exist "tests\coverage" mkdir tests\coverage

echo === Installing cargo-llvm-cov ===
cargo install cargo-llvm-cov

echo === Cleaning previous coverage data ===
cd sam
cargo llvm-cov clean

echo === Building SAM with coverage instrumentation ===
cargo llvm-cov --no-report --bin sam

echo === Running HIL tests ===
cd ..\tests\hil

REM Set environment for mock mode
set HIL_MODE=mock
set SAM_TARGET=localhost

REM Install Python dependencies if needed
if not exist "venv" (
    echo Creating Python virtual environment...
    python -m venv venv
)

call venv\Scripts\activate.bat

echo Installing Python dependencies...
pip install -r requirements.txt

echo Running pytest...
pytest -v --tb=short

echo === Generating coverage report ===
cd ..\..\sam

REM Generate coverage report
echo Generating LCOV report...
cargo llvm-cov report --lcov --output-path ..\tests\coverage\lcov.info

echo Generating HTML report...
cargo llvm-cov report --html --output-dir ..\tests\coverage\html

echo === Coverage report generated ===
echo LCOV report: tests\coverage\lcov.info
echo HTML report: tests\coverage\html\index.html

echo === Coverage script completed ===
