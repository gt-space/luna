"""
SAM-specific pytest configuration.
"""
import pytest
from ..common.flight_computer import FlightComputerClient
from ..common.mock_hardware import get_hardware_mock


@pytest.fixture
def sam_board_id() -> str:
    """Default SAM board identifier for testing."""
    return "sam-test-01"


@pytest.fixture
def sam_client(flight_computer_client: FlightComputerClient, sam_board_id: str) -> FlightComputerClient:
    """SAM-specific flight computer client with handshake."""
    # Perform handshake
    if not flight_computer_client.handshake(sam_board_id):
        pytest.skip("Could not establish connection with SAM")
    
    return flight_computer_client


@pytest.fixture
def hardware_mock():
    """Get hardware mock instance."""
    return get_hardware_mock()


@pytest.fixture(autouse=True)
def reset_hardware_mock(hardware_mock):
    """Reset hardware mock before each test."""
    hardware_mock.reset_all()
    yield
    hardware_mock.reset_all()
