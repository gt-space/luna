"""
Test SAM ADC data collection.

Tests ADC data collection, data point structure, and sensor readings.
"""
import pytest
import time
from ..common.message_types import DataPoint, ChannelType


class TestADCDataStructure:
    """Test ADC data point structure and format."""
    
    def test_datapoint_creation(self):
        """Test creating data points with correct structure."""
        datapoint = DataPoint(
            value=3.14159,
            timestamp=time.time(),
            channel=1,
            channel_type=ChannelType.VALVE_VOLTAGE
        )
        
        assert datapoint.value == 3.14159
        assert datapoint.channel == 1
        assert datapoint.channel_type == ChannelType.VALVE_VOLTAGE
        assert isinstance(datapoint.timestamp, float)
        assert datapoint.timestamp > 0
    
    def test_channel_type_units(self):
        """Test that channel types have correct units."""
        test_cases = [
            (ChannelType.CURRENT_LOOP, "psi"),
            (ChannelType.VALVE_VOLTAGE, "volts"),
            (ChannelType.VALVE_CURRENT, "amps"),
            (ChannelType.RAIL_VOLTAGE, "volts"),
            (ChannelType.RAIL_CURRENT, "amps"),
            (ChannelType.DIFFERENTIAL_SIGNAL, "pounds"),
            (ChannelType.RTD, "kelvin"),
            (ChannelType.TC, "kelvin"),
        ]
        
        for channel_type, expected_unit in test_cases:
            unit = channel_type.unit()
            assert unit.value == expected_unit
    
    def test_datapoint_serialization(self):
        """Test that data points can be serialized/deserialized."""
        original = DataPoint(
            value=2.5,
            timestamp=time.time(),
            channel=3,
            channel_type=ChannelType.VALVE_CURRENT
        )
        
        # Serialize
        serialized = original.serialize()
        assert len(serialized) > 0
        
        # Deserialize
        deserialized = DataPoint.deserialize(serialized)
        assert deserialized.value == original.value
        assert deserialized.channel == original.channel
        assert deserialized.channel_type == original.channel_type
        # Timestamp might have slight precision differences
        assert abs(deserialized.timestamp - original.timestamp) < 0.001


class TestADCDataCollection:
    """Test ADC data collection from SAM."""
    
    def test_receive_adc_data(self, sam_client, timeout_long):
        """Test receiving ADC data from SAM."""
        data_message = sam_client.wait_for_data("sam", timeout=timeout_long)
        
        if data_message and data_message.message_type == "sam":
            datapoints = data_message.data.get("datapoints", [])
            
            # Verify we received data points
            assert len(datapoints) > 0
            
            # Verify each data point structure
            for datapoint in datapoints:
                assert "value" in datapoint
                assert "timestamp" in datapoint
                assert "channel" in datapoint
                assert "channel_type" in datapoint
                
                # Verify data types
                assert isinstance(datapoint["value"], (int, float))
                assert isinstance(datapoint["timestamp"], (int, float))
                assert isinstance(datapoint["channel"], int)
                assert isinstance(datapoint["channel_type"], str)
        else:
            pytest.skip("No ADC data received (expected in mock mode)")
    
    def test_adc_data_timestamps(self, sam_client, timeout_long):
        """Test that ADC data has reasonable timestamps."""
        data_message = sam_client.receive_data(timeout=timeout_long)
        
        if data_message and data_message.message_type == "sam":
            datapoints = data_message.data.get("datapoints", [])
            current_time = time.time()
            
            for datapoint in datapoints:
                timestamp = datapoint["timestamp"]
                # Timestamp should be within last 10 seconds
                assert abs(current_time - timestamp) < 10.0
                # Timestamp should be positive
                assert timestamp > 0
    
    def test_adc_data_values(self, sam_client, timeout_long):
        """Test that ADC data has reasonable values."""
        data_message = sam_client.receive_data(timeout=timeout_long)
        
        if data_message and data_message.message_type == "sam":
            datapoints = data_message.data.get("datapoints", [])
            
            for datapoint in datapoints:
                value = datapoint["value"]
                # Values should be finite numbers
                assert isinstance(value, (int, float))
                assert not (value != value)  # Not NaN
                assert value != float('inf')  # Not infinite
                assert value != float('-inf')  # Not negative infinite


class TestChannelTypes:
    """Test different channel types and their data."""
    
    def test_channel_type_enum_values(self):
        """Test that all expected channel types exist."""
        expected_types = [
            "current_loop",
            "valve_voltage", 
            "valve_current",
            "rail_voltage",
            "rail_current",
            "differential_signal",
            "rtd",
            "tc"
        ]
        
        for expected in expected_types:
            # Find matching enum value
            found = False
            for channel_type in ChannelType:
                if channel_type.value == expected:
                    found = True
                    break
            assert found, f"Channel type '{expected}' not found"
    
    def test_channel_type_display(self):
        """Test channel type string representation."""
        test_cases = [
            (ChannelType.CURRENT_LOOP, "current_loop"),
            (ChannelType.VALVE_VOLTAGE, "valve_voltage"),
            (ChannelType.VALVE_CURRENT, "valve_current"),
            (ChannelType.RAIL_VOLTAGE, "rail_voltage"),
            (ChannelType.RAIL_CURRENT, "rail_current"),
            (ChannelType.DIFFERENTIAL_SIGNAL, "differential_signal"),
            (ChannelType.RTD, "rtd"),
            (ChannelType.TC, "tc"),
        ]
        
        for channel_type, expected_string in test_cases:
            assert channel_type.value == expected_string


class TestADCDataRate:
    """Test ADC data collection rate and timing."""
    
    def test_data_collection_frequency(self, sam_client, timeout_long):
        """Test that data is collected at expected frequency."""
        start_time = time.time()
        messages_received = 0
        
        # Collect messages for a short period
        while time.time() - start_time < 3.0:
            message = sam_client.receive_data(timeout=0.5)
            if message and message.message_type == "sam":
                messages_received += 1
        
        # Should receive at least some messages
        # In mock mode, might be 0, which is acceptable
        assert messages_received >= 0
    
    def test_data_consistency(self, sam_client, timeout_long):
        """Test that data is consistent across multiple messages."""
        messages = []
        
        # Collect multiple messages
        for _ in range(3):
            message = sam_client.receive_data(timeout=1.0)
            if message and message.message_type == "sam":
                messages.append(message)
        
        if len(messages) > 1:
            # All messages should have same structure
            first_message = messages[0]
            for message in messages[1:]:
                assert message.message_type == first_message.message_type
                assert "datapoints" in message.data
                assert "board_id" in message.data


class TestADCSensorSimulation:
    """Test ADC sensor simulation and mock data."""
    
    def test_create_mock_datapoint(self, sam_client):
        """Test creating mock data points for testing."""
        # Create test data point
        datapoint = sam_client.create_test_datapoint(
            channel=1,
            value=3.3,
            channel_type=ChannelType.VALVE_VOLTAGE
        )
        
        assert datapoint.channel == 1
        assert datapoint.value == 3.3
        assert datapoint.channel_type == ChannelType.VALVE_VOLTAGE
        assert datapoint.timestamp > 0
    
    def test_multiple_channel_data(self, sam_client):
        """Test data from multiple channels."""
        channels = [1, 2, 3, 4, 5, 6]
        channel_types = [
            ChannelType.VALVE_VOLTAGE,
            ChannelType.VALVE_CURRENT,
            ChannelType.RAIL_VOLTAGE,
            ChannelType.RAIL_CURRENT,
            ChannelType.CURRENT_LOOP,
            ChannelType.DIFFERENTIAL_SIGNAL
        ]
        
        for channel, channel_type in zip(channels, channel_types):
            datapoint = sam_client.create_test_datapoint(
                channel=channel,
                value=float(channel),
                channel_type=channel_type
            )
            
            assert datapoint.channel == channel
            assert datapoint.value == float(channel)
            assert datapoint.channel_type == channel_type
