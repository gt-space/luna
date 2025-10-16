"""
Global pytest configuration for HIL tests.
"""
import os
import pytest
import socket
import time
from typing import Generator, Tuple
from .common.flight_computer import FlightComputerClient


@pytest.fixture(scope="session")
def hil_mode() -> str:
    """Determine HIL mode from environment variable."""
    return os.getenv("HIL_MODE", "mock").lower()


@pytest.fixture(scope="session")
def sam_target() -> str:
    """SAM target address from environment or default."""
    return os.getenv("SAM_TARGET", "localhost")


@pytest.fixture(scope="session")
def data_port() -> int:
    """UDP port for data communication."""
    return 4573


@pytest.fixture(scope="session")
def command_port() -> int:
    """UDP port for command communication."""
    return 8378


@pytest.fixture(scope="session")
def flight_computer_client(
    sam_target: str, data_port: int, command_port: int
) -> Generator[FlightComputerClient, None, None]:
    """Create and cleanup flight computer client."""
    client = FlightComputerClient(sam_target, data_port, command_port)
    yield client
    client.close()


@pytest.fixture
def timeout_short() -> float:
    """Short timeout for quick operations."""
    return 1.0


@pytest.fixture
def timeout_long() -> float:
    """Long timeout for slow operations."""
    return 10.0


@pytest.fixture(autouse=True)
def setup_test_environment():
    """Setup test environment before each test."""
    # Ensure we're in a clean state
    time.sleep(0.1)  # Brief pause to avoid race conditions
