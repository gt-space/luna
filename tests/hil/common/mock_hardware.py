"""
Hardware mocking layer for HIL testing.

Provides mock implementations of GPIO and SPI interfaces
for CI/CD testing without real hardware.
"""
import os
from typing import Dict, Any, Optional, Tuple, List
from unittest.mock import Mock, MagicMock


class MockGPIOController:
    """Mock GPIO controller for testing."""
    
    def __init__(self, controller_id: int):
        self.controller_id = controller_id
        self.pins: Dict[int, MockPin] = {}
        self._initialized = False
    
    def get_pin(self, pin_num: int) -> 'MockPin':
        """Get a mock pin."""
        if pin_num not in self.pins:
            self.pins[pin_num] = MockPin(pin_num, self.controller_id)
        return self.pins[pin_num]
    
    def initialize(self):
        """Initialize the controller."""
        self._initialized = True
    
    def is_initialized(self) -> bool:
        """Check if controller is initialized."""
        return self._initialized


class MockPin:
    """Mock GPIO pin for testing."""
    
    def __init__(self, pin_num: int, controller_id: int):
        self.pin_num = pin_num
        self.controller_id = controller_id
        self.mode = None
        self.value = None
        self._call_history = []
    
    def mode(self, mode):
        """Set pin mode."""
        self.mode = mode
        self._call_history.append(("mode", mode))
    
    def digital_write(self, value):
        """Write digital value to pin."""
        self.value = value
        self._call_history.append(("digital_write", value))
    
    def digital_read(self):
        """Read digital value from pin."""
        return self.value
    
    def get_call_history(self):
        """Get history of calls made to this pin."""
        return self._call_history.copy()


class MockSPI:
    """Mock SPI interface for testing."""
    
    def __init__(self, bus: int):
        self.bus = bus
        self._transactions = []
        self._is_open = False
    
    def open(self):
        """Open SPI bus."""
        self._is_open = True
    
    def close(self):
        """Close SPI bus."""
        self._is_open = False
    
    def transfer(self, data: bytes) -> bytes:
        """Mock SPI transfer."""
        self._transactions.append(("transfer", data))
        # Return mock response
        return b'\x00' * len(data)
    
    def is_open(self) -> bool:
        """Check if SPI is open."""
        return self._is_open
    
    def get_transactions(self):
        """Get history of SPI transactions."""
        return self._transactions.copy()


class HardwareMock:
    """Main hardware mocking class."""
    
    def __init__(self):
        self.mode = os.getenv("HIL_MODE", "mock").lower()
        self.gpio_controllers: Dict[int, MockGPIOController] = {}
        self.spi_buses: Dict[int, MockSPI] = {}
        self._mock_enabled = self.mode == "mock"
    
    def is_mock_mode(self) -> bool:
        """Check if running in mock mode."""
        return self._mock_enabled
    
    def get_gpio_controller(self, controller_id: int) -> MockGPIOController:
        """Get a mock GPIO controller."""
        if controller_id not in self.gpio_controllers:
            self.gpio_controllers[controller_id] = MockGPIOController(controller_id)
        return self.gpio_controllers[controller_id]
    
    def get_spi_bus(self, bus: int) -> MockSPI:
        """Get a mock SPI bus."""
        if bus not in self.spi_buses:
            self.spi_buses[bus] = MockSPI(bus)
        return self.spi_buses[bus]
    
    def reset_all(self):
        """Reset all mock hardware to initial state."""
        for controller in self.gpio_controllers.values():
            controller.pins.clear()
        for spi in self.spi_buses.values():
            spi._transactions.clear()
    
    def get_all_pin_states(self) -> Dict[Tuple[int, int], Dict[str, Any]]:
        """Get current state of all pins."""
        states = {}
        for controller_id, controller in self.gpio_controllers.items():
            for pin_num, pin in controller.pins.items():
                states[(controller_id, pin_num)] = {
                    "mode": pin.mode,
                    "value": pin.value,
                    "calls": pin.get_call_history()
                }
        return states
    
    def get_all_spi_transactions(self) -> Dict[int, List[Tuple[str, Any]]]:
        """Get all SPI transaction history."""
        transactions = {}
        for bus, spi in self.spi_buses.items():
            transactions[bus] = spi.get_transactions()
        return transactions


# Global hardware mock instance
hardware_mock = HardwareMock()


def get_hardware_mock() -> HardwareMock:
    """Get the global hardware mock instance."""
    return hardware_mock


def mock_gpio_if_needed():
    """Apply GPIO mocking if in mock mode."""
    if hardware_mock.is_mock_mode():
        # This would be called by the test framework to patch GPIO calls
        # In a real implementation, this would use unittest.mock.patch
        # to intercept GPIO library calls
        pass


def mock_spi_if_needed():
    """Apply SPI mocking if in mock mode."""
    if hardware_mock.is_mock_mode():
        # This would be called by the test framework to patch SPI calls
        # In a real implementation, this would use unittest.mock.patch
        # to intercept SPI library calls
        pass
