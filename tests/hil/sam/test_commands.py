"""
Test SAM command execution.

Tests valve actuation commands and command validation.
"""
import pytest
import time
from ..common.message_types import SamControlMessage, ChannelType


class TestValveCommands:
    """Test valve actuation commands."""
    
    def test_actuate_valve_power_on(self, sam_client, timeout_short):
        """Test powering on a valve."""
        # Send command to power on valve 1
        command = SamControlMessage.actuate_valve(channel=1, powered=True)
        assert sam_client.send_command(command)
        
        # Give SAM time to process command
        time.sleep(timeout_short)
        
        # In mock mode, verify the command was sent
        # In real mode, we'd verify actual hardware state
        # For now, just verify no exceptions occurred
        assert True
    
    def test_actuate_valve_power_off(self, sam_client, timeout_short):
        """Test powering off a valve."""
        # Send command to power off valve 1
        command = SamControlMessage.actuate_valve(channel=1, powered=False)
        assert sam_client.send_command(command)
        
        time.sleep(timeout_short)
        assert True
    
    def test_actuate_multiple_valves(self, sam_client, timeout_short):
        """Test actuating multiple valves in sequence."""
        valves_to_test = [1, 2, 3]
        
        for valve in valves_to_test:
            # Power on
            command = SamControlMessage.actuate_valve(channel=valve, powered=True)
            assert sam_client.send_command(command)
            time.sleep(0.1)
            
            # Power off
            command = SamControlMessage.actuate_valve(channel=valve, powered=False)
            assert sam_client.send_command(command)
            time.sleep(0.1)
    
    @pytest.mark.parametrize("valve_channel", [1, 2, 3, 4, 5, 6])
    def test_all_valve_channels(self, sam_client, valve_channel, timeout_short):
        """Test all valid valve channels."""
        command = SamControlMessage.actuate_valve(channel=valve_channel, powered=True)
        assert sam_client.send_command(command)
        time.sleep(timeout_short)
        
        # Turn off
        command = SamControlMessage.actuate_valve(channel=valve_channel, powered=False)
        assert sam_client.send_command(command)
        time.sleep(timeout_short)
    
    def test_invalid_valve_channel(self, sam_client):
        """Test that invalid valve channels are handled gracefully."""
        # Test channel 0 (invalid)
        command = SamControlMessage.actuate_valve(channel=0, powered=True)
        # SAM should handle this gracefully, not crash
        assert sam_client.send_command(command)
        
        # Test channel 7 (invalid)
        command = SamControlMessage.actuate_valve(channel=7, powered=True)
        assert sam_client.send_command(command)
    
    def test_rapid_valve_commands(self, sam_client):
        """Test rapid succession of valve commands."""
        # Send multiple commands quickly
        for i in range(10):
            command = SamControlMessage.actuate_valve(channel=1, powered=(i % 2 == 0))
            assert sam_client.send_command(command)
            time.sleep(0.05)  # 50ms between commands


class TestCommandValidation:
    """Test command validation and error handling."""
    
    def test_command_serialization(self):
        """Test that commands can be serialized properly."""
        command = SamControlMessage.actuate_valve(channel=1, powered=True)
        serialized = command.serialize()
        assert len(serialized) > 0
        
        # Verify we can deserialize
        deserialized = SamControlMessage.deserialize(serialized)
        assert deserialized.command_type == "actuate_valve"
        assert deserialized.channel == 1
        assert deserialized.powered is True
    
    def test_multiple_command_types(self, sam_client, timeout_short):
        """Test sending multiple different command types."""
        # Test different valve states
        test_cases = [
            (1, True),
            (2, False),
            (3, True),
            (4, False),
        ]
        
        for channel, powered in test_cases:
            command = SamControlMessage.actuate_valve(channel=channel, powered=powered)
            assert sam_client.send_command(command)
            time.sleep(0.1)
    
    def test_command_with_connection_loss(self, sam_client):
        """Test command handling when connection is lost."""
        # This test would simulate connection loss
        # In a real scenario, we'd close the connection and verify
        # that SAM handles the disconnection gracefully
        pass
