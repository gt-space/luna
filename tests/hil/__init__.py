"""
Hardware-in-the-Loop (HIL) Testing Framework

This package provides a testing framework for embedded Rust systems that communicate
via UDP. Tests act as a "fake flight computer" to validate system behavior without
modifying the target code.

Supports two modes:
- mock: For CI/CD pipelines with simulated hardware
- real: For testing on actual hardware
"""

__version__ = "0.1.0"

