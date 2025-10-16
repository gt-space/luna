"""
Fake Flight Computer UDP Client for HIL testing.

This module provides a client that acts as a flight computer,
communicating with SAM via UDP to test the complete system.
"""
import socket
import time
import threading
import json
from typing import Optional, Tuple, List
from .message_types import DataMessage, SamControlMessage, DataPoint, ChannelType


class FlightComputerClient:
    """UDP client that acts as a fake flight computer."""
    
    def __init__(self, sam_target: str, data_port: int, command_port: int):
        """Initialize the flight computer client."""
        self.sam_target = sam_target
        self.data_port = data_port
        self.command_port = command_port
        
        # Create UDP sockets
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Bind to receive data from SAM
        self.data_socket.bind(("0.0.0.0", data_port))
        self.command_socket.bind(("0.0.0.0", command_port))
        
        # Set non-blocking for async operations
        self.data_socket.setblocking(False)
        self.command_socket.setblocking(False)
        
        # Heartbeat thread
        self._heartbeat_thread = None
        self._heartbeat_running = False
        self._connected = False
        
    def close(self):
        """Close connections and cleanup."""
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=1.0)
        self.data_socket.close()
        self.command_socket.close()
    
    def handshake(self, board_id: str, timeout: float = 5.0) -> bool:
        """
        Perform handshake with SAM board.
        
        Args:
            board_id: The board identifier to send
            timeout: Maximum time to wait for response
            
        Returns:
            True if handshake successful, False otherwise
        """
        # Send identity message
        identity_msg = DataMessage.identity(board_id)
        identity_data = identity_msg.serialize()
        
        # Send to SAM's data port
        self.data_socket.sendto(identity_data, (self.sam_target, self.data_port))
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = self.data_socket.recvfrom(1024)
                if data:  # Only try to deserialize if we got data
                    response = DataMessage.deserialize(data)
                    
                    if response.message_type == "identity":
                        self._connected = True
                        self._start_heartbeat()
                        return True
                    
            except (socket.error, json.JSONDecodeError, UnicodeDecodeError):
                time.sleep(0.1)
                continue
                
        return False
    
    def _start_heartbeat(self):
        """Start heartbeat thread to keep SAM alive."""
        if self._heartbeat_running:
            return
            
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()
    
    def _heartbeat_loop(self):
        """Send periodic heartbeats to SAM."""
        while self._heartbeat_running:
            try:
                heartbeat = DataMessage.flight_heartbeat()
                heartbeat_data = heartbeat.serialize()
                self.data_socket.sendto(heartbeat_data, (self.sam_target, self.data_port))
                time.sleep(0.5)  # Send heartbeat every 500ms
            except Exception:
                break
    
    def send_command(self, command: SamControlMessage) -> bool:
        """
        Send a command to SAM.
        
        Args:
            command: The command to send
            
        Returns:
            True if command sent successfully
        """
        try:
            command_data = command.serialize()
            self.command_socket.sendto(command_data, (self.sam_target, self.command_port))
            return True
        except Exception:
            return False
    
    def receive_data(self, timeout: float = 1.0) -> Optional[DataMessage]:
        """
        Receive data from SAM.
        
        Args:
            timeout: Maximum time to wait for data
            
        Returns:
            DataMessage if received, None if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = self.data_socket.recvfrom(2048)
                message = DataMessage.deserialize(data)
                return message
            except socket.error:
                time.sleep(0.01)
                continue
        return None
    
    def wait_for_data(self, expected_type: str, timeout: float = 5.0) -> Optional[DataMessage]:
        """
        Wait for specific type of data from SAM.
        
        Args:
            expected_type: Expected message type ("sam", "bms", "ahrs")
            timeout: Maximum time to wait
            
        Returns:
            DataMessage if received, None if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.receive_data(timeout=0.1)
            if message and message.message_type == expected_type:
                return message
        return None
    
    def is_connected(self) -> bool:
        """Check if connected to SAM."""
        return self._connected
    
    def create_test_datapoint(self, channel: int, value: float, 
                            channel_type: ChannelType) -> DataPoint:
        """Create a test data point."""
        return DataPoint(
            value=value,
            timestamp=time.time(),
            channel=channel,
            channel_type=channel_type
        )
