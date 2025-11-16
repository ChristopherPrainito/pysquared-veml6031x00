"""Mock for the Adafruit DRV2605L haptic motor driver.

This module provides a mock implementation of the Adafruit DRV2605L driver for
testing purposes. It allows for simulating the behavior of the DRV2605L without the
need for actual hardware.
"""

from busio import I2C


class DRV2605:
    """A mock DRV2605 haptic motor driver."""

    def __init__(self, i2c: I2C, address: int = 0x5A) -> None:
        """Initializes the mock DRV2605.

        Args:
            i2c: The I2C bus to use.
            address: The I2C address of the DRV2605L.
        """
        self.i2c = i2c
        self.address = address
        self.mode = 0x00
        self.realtime_value = 0
        self._library = 1  # Default ERM library

    def use_ERM(self) -> None:
        """Configures the driver to use ERM (Eccentric Rotating Mass) mode."""
        self._library = 1

    def use_LRA(self) -> None:
        """Configures the driver to use LRA (Linear Resonant Actuator) mode."""
        self._library = 6
