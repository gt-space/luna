"""
Python message types matching Rust communication structures.

This module provides Python classes that mirror the Rust message types
defined in common/src/comm/ for UDP communication with SAM.
"""
from dataclasses import dataclass
from enum import Enum
from typing import List, Union, Dict, Any
import json


class Unit(Enum):
    """Units for sensor readings."""
    AMPS = "amps"
    PSI = "psi"
    KELVIN = "kelvin"
    POUNDS = "pounds"
    VOLTS = "volts"


class ChannelType(Enum):
    """Channel types for data points."""
    CURRENT_LOOP = "current_loop"
    VALVE_VOLTAGE = "valve_voltage"
    VALVE_CURRENT = "valve_current"
    RAIL_VOLTAGE = "rail_voltage"
    RAIL_CURRENT = "rail_current"
    DIFFERENTIAL_SIGNAL = "differential_signal"
    RTD = "rtd"
    TC = "tc"

    def unit(self) -> Unit:
        """Get the associated unit for this channel type."""
        unit_map = {
            self.CURRENT_LOOP: Unit.PSI,
            self.VALVE_VOLTAGE: Unit.VOLTS,
            self.VALVE_CURRENT: Unit.AMPS,
            self.RAIL_VOLTAGE: Unit.VOLTS,
            self.RAIL_CURRENT: Unit.AMPS,
            self.DIFFERENTIAL_SIGNAL: Unit.POUNDS,
            self.RTD: Unit.KELVIN,
            self.TC: Unit.KELVIN,
        }
        return unit_map[self]


@dataclass
class DataPoint:
    """A single data point with timestamp and channel."""
    value: float
    timestamp: float
    channel: int
    channel_type: ChannelType
    
    def serialize(self) -> bytes:
        """Serialize to JSON bytes."""
        data = {
            "value": self.value,
            "timestamp": self.timestamp,
            "channel": self.channel,
            "channel_type": self.channel_type.value
        }
        return json.dumps(data).encode('utf-8')
    
    @classmethod
    def deserialize(cls, data: bytes) -> 'DataPoint':
        """Deserialize from JSON bytes."""
        obj = json.loads(data.decode('utf-8'))
        return cls(
            value=obj["value"],
            timestamp=obj["timestamp"],
            channel=obj["channel"],
            channel_type=ChannelType(obj["channel_type"])
        )


@dataclass
class SamControlMessage:
    """Control message from flight computer to SAM."""
    command_type: str
    channel: int = None
    powered: bool = None

    @classmethod
    def actuate_valve(cls, channel: int, powered: bool) -> 'SamControlMessage':
        """Create a valve actuation command."""
        return cls(
            command_type="actuate_valve",
            channel=channel,
            powered=powered
        )
    
    def serialize(self) -> bytes:
        """Serialize to JSON bytes."""
        data = {
            "command_type": self.command_type,
            "channel": self.channel,
            "powered": self.powered
        }
        return json.dumps(data).encode('utf-8')
    
    @classmethod
    def deserialize(cls, data: bytes) -> 'SamControlMessage':
        """Deserialize from JSON bytes."""
        obj = json.loads(data.decode('utf-8'))
        return cls(
            command_type=obj["command_type"],
            channel=obj.get("channel"),
            powered=obj.get("powered")
        )


class DataMessage:
    """Data message enum for flight computer communication."""
    
    def __init__(self, message_type: str, **kwargs):
        self.message_type = message_type
        self.data = kwargs

    @classmethod
    def identity(cls, board_id: str) -> 'DataMessage':
        """Create an identity handshake message."""
        return cls("identity", board_id=board_id)

    @classmethod
    def flight_heartbeat(cls) -> 'DataMessage':
        """Create a flight computer heartbeat."""
        return cls("flight_heartbeat")

    @classmethod
    def sam_data(cls, board_id: str, datapoints: List[DataPoint]) -> 'DataMessage':
        """Create SAM data message."""
        return cls("sam", board_id=board_id, datapoints=datapoints)

    @classmethod
    def bms_data(cls, board_id: str, datapoint: DataPoint) -> 'DataMessage':
        """Create BMS data message."""
        return cls("bms", board_id=board_id, datapoint=datapoint)

    @classmethod
    def ahrs_data(cls, board_id: str, datapoints: List[DataPoint]) -> 'DataMessage':
        """Create AHRS data message."""
        return cls("ahrs", board_id=board_id, datapoints=datapoints)
    
    def serialize(self) -> bytes:
        """Serialize to JSON bytes."""
        data = {
            "message_type": self.message_type,
            **self.data
        }
        return json.dumps(data).encode('utf-8')
    
    @classmethod
    def deserialize(cls, data: bytes) -> 'DataMessage':
        """Deserialize from JSON bytes."""
        obj = json.loads(data.decode('utf-8'))
        message_type = obj.pop("message_type")
        return cls(message_type, **obj)


def serialize_message(message: Union[DataMessage, SamControlMessage]) -> bytes:
    """Serialize a message to JSON format."""
    return message.serialize()


def deserialize_message(data: bytes, message_type: str) -> Union[DataMessage, SamControlMessage]:
    """Deserialize a message from JSON format."""
    if message_type == "data":
        return DataMessage.deserialize(data)
    elif message_type == "control":
        return SamControlMessage.deserialize(data)
    else:
        raise ValueError(f"Unknown message type: {message_type}")
