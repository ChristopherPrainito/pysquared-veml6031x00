"""VEML6031X00 Light Sensor Manager
================================================================================

This module provides a complete implementation for the VEML6031X00 ambient light sensor,
including both the low-level driver and high-level manager interface.

The VEML6031X00 is a high precision I2C ambient light sensor from Vishay with configurable
gain and integration time settings.

**Hardware:**
* Vishay VEML6031X00 Lux Sensor - I2C Light Sensor
  <https://www.vishay.com/en/product/80007/>

**Authors:** Christopher Prainito, Madison Davis
**Repository:** https://github.com/Harvard-Satellite-Team-Ground-Station/OBC_v5d
"""

import time

import adafruit_bus_device.i2c_device as i2cdevice
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_tca9548a import TCA9548A_Channel
from busio import I2C
from micropython import const

from ....logger import Logger
from ....protos.light_sensor import LightSensorProto
from ....sensor_reading.error import (
    SensorReadingUnknownError,
    SensorReadingValueError,
)
from ....sensor_reading.light import Light
from ....sensor_reading.lux import Lux
from ...exception import HardwareInitializationError

try:
    from typing import Literal
except ImportError:
    pass


# ============================================================================
# Low-Level Driver: VEML6031X00
# ============================================================================


class _VEML6031X00:
    """Low-level driver for the VEML6031X00 ambient light sensor.

    This internal class handles direct I2C communication with the sensor hardware.
    Users should interact with the high-level VEML6031X00Manager instead.

    :param ~busio.I2C i2c_bus: The I2C bus the device is connected to
    :param int address: The I2C device address. Defaults to :const:`0x29`
    """

    # Ambient light sensor gain settings
    ALS_GAIN_1 = const(0x0)
    ALS_GAIN_2 = const(0x1)
    ALS_GAIN_2_3 = const(0x2)
    ALS_GAIN_1_2 = const(0x3)

    # Ambient light integration time settings
    ALS_3_125MS = const(0x0)
    ALS_6_25MS = const(0x1)
    ALS_12_5MS = const(0x2)
    ALS_25MS = const(0x3)
    ALS_50MS = const(0x4)
    ALS_100MS = const(0x5)
    ALS_200MS = const(0x6)
    ALS_400MS = const(0x7)

    # Gain value mappings
    gain_values = {
        ALS_GAIN_2: 2,
        ALS_GAIN_1: 1,
        ALS_GAIN_2_3: 0.66,
        ALS_GAIN_1_2: 0.5,
    }

    # Integration time value mappings (in milliseconds)
    integration_time_values = {  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_3_125MS: 3.125,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_6_25MS: 6.25,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_12_5MS: 12.5,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_25MS: 25,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_50MS: 50,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_100MS: 100,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_200MS: 200,  # noqa: codespell  # ALS = Ambient Light Sensor
        ALS_400MS: 400,  # noqa: codespell  # ALS = Ambient Light Sensor
    }

    # ALS - Ambient light sensor high resolution output data  # noqa: codespell
    light = ROUnaryStruct(0x04, "<H")
    """Ambient light data.

    This example prints the ambient light data. Cover the sensor to see the values change.

    .. code-block:: python

        import time
        import board
        from pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031X00Manager

        i2c = board.I2C()  # uses board.SCL and board.SDA
        light_sensor = VEML6031X00Manager(i2c)

        while True:
            light_reading = light_sensor.get_light()
            print("Ambient light:", light_reading.light)
            time.sleep(0.1)
    """

    # WHITE - White channel output data
    white = ROUnaryStruct(0x05, "<H")
    """White light data.

    This example prints the white light data. Cover the sensor to see the values change.

    .. code-block:: python

        import time
        import board
        from pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031X00Manager

        i2c = board.I2C()  # uses board.SCL and board.SDA
        light_sensor = VEML6031X00Manager(i2c)

        while True:
            # Access internal driver for white channel (advanced usage)
            print("White light:", light_sensor._light_sensor.white)
            time.sleep(0.1)
    """

    # ALS_CONF_0 - ALS gain, integration time, shutdown.  # noqa: codespell
    # Full 16-bit configuration register for atomic writes
    als_conf_0 = UnaryStruct(0x00, "<H")  # noqa: codespell
    """Full 16-bit ALS configuration register for atomic writes."""
    
    # Individual bit/field accessors (for reading/convenience)
    light_shutdown = RWBit(0x00, 0, register_width=2)
    """Ambient light sensor shutdown. When ``True``, ambient light sensor is disabled."""
    light_interrupt_enable = RWBit(0x00, 1, register_width=2)
    """Ambient light interrupt enable. When ``True``, interrupt is enabled."""
    light_active_force_trigger = RWBit(0x00, 2, register_width=2)
    """Active force mode trigger. Set to ``True`` to trigger a single measurement."""
    light_active_force_enable = RWBit(0x00, 3, register_width=2)
    """Active force mode enable. When ``True``, sensor operates in single-shot mode."""
    light_gain = RWBits(2, 0x00, 11, register_width=2)
    """Ambient light gain setting. Gain settings are 2, 1, 2/3 and 1/2. Settings options are:
    ALS_GAIN_2, ALS_GAIN_1, ALS_GAIN_2_3, ALS_GAIN_1_2.

    This example sets the ambient light gain to 2 and prints the ambient light sensor data.

    .. code-block:: python

        import time
        import board
        from pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031X00Manager

        i2c = board.I2C()  # uses board.SCL and board.SDA
        light_sensor = VEML6031X00Manager(i2c)

        # Access internal driver for gain control (advanced usage)
        light_sensor._light_sensor.light_gain = light_sensor._light_sensor.ALS_GAIN_2

        while True:
            lux_reading = light_sensor.get_lux()
            print("Ambient light:", lux_reading.lux)
            time.sleep(0.1)

    """
    light_integration_time = RWBits(4, 0x00, 6, register_width=2)
    """Ambient light integration time setting. Longer time has higher sensitivity. Can be:
    ALS_3_125MS, ALS_6_25MS, ALS_12_5MS, ALS_25MS, ALS_50MS, ALS_100MS, ALS_200MS, ALS_400MS.

    This example sets the ambient light integration time to 400ms and prints the ambient light
    sensor data.

    .. code-block:: python

        import time
        import board
        from pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031X00Manager

        i2c = board.I2C()  # uses board.SCL and board.SDA
        light_sensor = VEML6031X00Manager(i2c)

        # Access internal driver for integration time control (advanced usage)
        light_sensor._light_sensor.light_integration_time = light_sensor._light_sensor.ALS_400MS

        while True:
            lux_reading = light_sensor.get_lux()
            print("Ambient light:", lux_reading.lux)
            time.sleep(0.1)

    """

    # ALS_INT - Interrupt status register
    als_int_data_ready = RWBit(0x17, 3, register_width=1)  # noqa: codespell  # ALS = Ambient Light Sensor
    """Data ready flag in active force mode. When ``True``, measurement is complete."""

    def __init__(self, i2c_bus: I2C, address: int = 0x29) -> None:
        """Initialize the VEML6031X00 sensor.

        Args:
            i2c_bus: The I2C bus connected to the sensor
            address: The I2C device address (default: 0x29)

        Raises:
            RuntimeError: If unable to initialize the sensor after 3 attempts
        """
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, address)

        # Try to initialize the sensor with retry logic
        for _ in range(3):
            try:
                # Set lowest gain to prevent overflow in bright light
                self.light_gain = self.ALS_GAIN_1_2
                # Enable active force mode for single-shot measurements
                self.light_active_force_enable = True
                # Disable interrupt
                self.light_interrupt_enable = False
                # Enable the ambient light sensor
                self.light_shutdown = False
                break
            except OSError:
                pass
        else:
            raise RuntimeError("Unable to enable VEML6031X00 device")

    def trigger_measurement(self) -> None:
        """Trigger a single measurement in active force mode.

        This writes the entire configuration register atomically (like the C driver)
        to ensure proper sensor operation. The configuration includes:
        - Integration time (from current setting)
        - Gain (from current setting)  
        - Active force mode enabled
        - Active force trigger set
        - Interrupts disabled
        - Sensor not shutdown
        """
        # Build configuration word matching C driver's veml6031_write_conf()
        # Low byte (conf[0] in C driver):
        # Bit 0: SD (shutdown) = 0
        # Bit 1: INT_EN (interrupt enable) = 0
        # Bit 2: TRIG (trigger) = 1
        # Bit 3: AF (active force enable) = 1
        # Bits 6:4: IT (integration time) from current setting
        
        # High byte (conf[1] in C driver):
        # Bit 0: PON (power on ready) = 0
        # Bits 2:1: PERS (persistence) = 0
        # Bits 4:3: GAIN from current setting
        # Bit 5: reserved = 0
        # Bit 6: DIV4 = 0
        # Bit 7: IR_SD = 0
        
        current_integration = self.light_integration_time
        current_gain = self.light_gain
        
        # Build 16-bit config word: [high_byte][low_byte]
        low_byte = (
            0x00  # Bit 0: SD = 0 (not shutdown)
            | 0x00  # Bit 1: INT_EN = 0 (interrupt disabled)
            | 0x04  # Bit 2: TRIG = 1 (trigger measurement)
            | 0x08  # Bit 3: AF = 1 (active force enabled)
            | (current_integration << 4)  # Bits 6:4: integration time
        )
        
        high_byte = (
            0x00  # Bit 0: PON = 0
            | 0x00  # Bits 2:1: PERS = 0
            | (current_gain << 3)  # Bits 4:3: gain
        )
        
        config_word = (high_byte << 8) | low_byte
        
        # Write entire config register atomically (single I2C transaction)
        self.als_conf_0 = config_word  # noqa: codespell

    def wait_for_data_ready(self, timeout_ms: int = 1000) -> bool:
        """Wait for measurement to complete by polling data ready flag.

        Args:
            timeout_ms: Maximum time to wait in milliseconds (default: 1000ms)
        Returns:
            bool: True if data is ready, False if timeout occurred
        """
        # First wait for the integration time
        integration_time_ms = self.integration_time_value()
        time.sleep(integration_time_ms / 1000.0)

        # Then poll the data ready flag
        start_time = time.monotonic()
        timeout_s = timeout_ms / 1000.0

        while (time.monotonic() - start_time) < timeout_s:
            if self.als_int_data_ready:
                return True
            time.sleep(0.001)  # 1ms polling interval

        return False

    def read_light_with_trigger(self) -> int:
        """Perform a triggered measurement and return raw light data.

        This is the recommended way to read from the sensor as it ensures
        fresh data by triggering a new measurement and waiting for completion.

        Returns:
            int: Raw light sensor reading in counts

        Raises:
            RuntimeError: If measurement times out or fails
        """
        # Trigger new measurement
        self.trigger_measurement()

        # Wait for data ready
        if not self.wait_for_data_ready():
            raise RuntimeError("Timeout waiting for VEML6031X00 data ready")

        # Read the data
        return self.light

    def integration_time_value(self) -> float:
        """Get the current integration time value in milliseconds.

        Returns:
            float: Integration time in milliseconds
        """
        integration_time = self.light_integration_time
        return self.integration_time_values[integration_time]

    def gain_value(self) -> float:
        """Get the current gain multiplier value.

        Returns:
            float: Gain multiplier (2, 1, 0.66, or 0.5)
        """
        gain = self.light_gain
        return self.gain_values[gain]

    def resolution(self) -> float:
        """Calculate the lux resolution based on current gain and integration time.

        The resolution determines how to convert raw sensor readings to lux values.
        It's calculated relative to the maximum sensitivity settings (gain=2, integration=400ms).

        Returns:
            float: Resolution factor for lux calculation
        """
        resolution_at_max = 0.0034
        gain_max = 2
        integration_time_max = 400

        if (
            self.gain_value() == gain_max
            and self.integration_time_value() == integration_time_max
        ):
            return resolution_at_max

        return (
            resolution_at_max
            * (integration_time_max / self.integration_time_value())
            * (gain_max / self.gain_value())
        )

    @property
    def lux(self) -> float:
        """Light value in lux.

        This example prints the light data in lux. Cover the sensor to see the values change.

        .. code-block:: python

            import time
            import board
            from pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031X00Manager

            i2c = board.I2C()  # uses board.SCL and board.SDA
            light_sensor = VEML6031X00Manager(i2c)

            while True:
                lux_reading = light_sensor.get_lux()
                print("Lux:", lux_reading.lux)
                time.sleep(0.1)
        """
        light_reading = self.read_light_with_trigger()
        return self.resolution() * light_reading

    @property
    def autolux(self) -> float:
        """Lux value with auto-gain and auto-integration time.

        This method automatically adjusts the gain and integration time to find
        the optimal settings for the current lighting conditions, then returns
        the lux value.

        Returns:
            float: The calculated lux value with optimal settings.
        """
        # Store original settings to restore later
        original_gain = self.light_gain
        original_integration = self.light_integration_time

        # Test configurations from highest to lowest sensitivity
        test_configs = [
            (self.ALS_GAIN_2, self.ALS_400MS),
            (self.ALS_GAIN_1, self.ALS_400MS),
            (self.ALS_GAIN_2_3, self.ALS_200MS),
            (self.ALS_GAIN_1_2, self.ALS_100MS),
        ]

        best_lux = None
        for gain, integration in test_configs:
            self.light_gain = gain
            self.light_integration_time = integration

            # Trigger measurement and wait for data ready
            try:
                reading = self.read_light_with_trigger()
                # Check if reading is in valid range (not saturated, not too low)
                if 100 < reading < 60000:
                    best_lux = self.resolution() * reading
                    break
            except RuntimeError:
                # If measurement times out, try next configuration
                continue

        # Use last configuration if no optimal reading found
        if best_lux is None:
            try:
                reading = self.read_light_with_trigger()
                best_lux = self.resolution() * reading
            except RuntimeError:
                # If still failing, return 0
                best_lux = 0.0

        # Restore original settings
        self.light_gain = original_gain
        self.light_integration_time = original_integration

        return best_lux


# ============================================================================
# High-Level Manager: VEML6031X00Manager
# ============================================================================


class VEML6031X00Manager(LightSensorProto):
    """High-level manager for the VEML6031X00 ambient light sensor.

    This class provides a clean interface for sensor operations with proper error
    handling, logging, and integration with the pysquared framework.
    """

    def __init__(
        self,
        logger: Logger,
        i2c: I2C | TCA9548A_Channel,
        integration_time: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 3,
    ) -> None:
        """Initialize the VEML6031X00Manager.

        Args:
            logger: Logger instance for debug/error messages
            i2c: I2C bus or TCA9548A channel connected to the sensor
            integration_time: Integration time setting (default is 3 = 25ms)
                - 0: 3.125ms
                - 1: 6.25ms
                - 2: 12.5ms
                - 3: 25ms (default)
                - 4: 50ms
                - 5: 100ms
                - 6: 200ms
                - 7: 400ms

        Raises:
            HardwareInitializationError: If the sensor fails to initialize
        """
        self._log: Logger = logger
        self._i2c: I2C | TCA9548A_Channel = i2c

        try:
            self._log.debug("Initializing VEML6031X00 light sensor")
            self._light_sensor = _VEML6031X00(i2c)
            self._light_sensor.light_integration_time = integration_time
        except Exception as e:
            raise HardwareInitializationError(
                "Failed to initialize VEML6031X00 light sensor"
            ) from e

    def get_light(self) -> Light:
        """Get raw light reading from the sensor.

        Returns:
            Light: Object containing non-unit-specific light level reading

        Raises:
            SensorReadingUnknownError: If sensor communication fails
        """
        try:
            # Use triggered measurement for fresh data
            light_value = self._light_sensor.read_light_with_trigger()
            return Light(light_value)
        except Exception as e:
            raise SensorReadingUnknownError("Failed to get light reading") from e

    def get_lux(self) -> Lux:
        """Get calibrated lux reading from the sensor.

        Returns:
            Lux: Object containing light level in SI lux units

        Raises:
            SensorReadingValueError: If reading is invalid or zero
            SensorReadingUnknownError: If sensor communication fails
        """
        try:
            lux = self._light_sensor.lux
        except Exception as e:
            raise SensorReadingUnknownError("Failed to get lux reading") from e

        if self._is_invalid_lux(lux):
            raise SensorReadingValueError("Lux reading is invalid or zero")

        return Lux(lux)

    def get_auto_lux(self) -> Lux:
        """Get lux reading with automatic gain and integration time optimization.

        The sensor automatically adjusts settings to find the best configuration
        for current lighting conditions, avoiding saturation and low readings.

        Returns:
            Lux: Object containing optimized light level in SI lux units

        Raises:
            SensorReadingValueError: If reading is invalid or zero
            SensorReadingUnknownError: If sensor communication fails
        """
        try:
            lux = self._light_sensor.autolux
        except Exception as e:
            raise SensorReadingUnknownError("Failed to get auto lux reading") from e

        if self._is_invalid_lux(lux):
            raise SensorReadingValueError("Lux reading is invalid or zero")

        return Lux(lux)

    @staticmethod
    def _is_invalid_lux(lux: float | None) -> bool:
        """Check if a lux reading is invalid.

        Args:
            lux: The lux reading to validate (can be None or a float)

        Returns:
            bool: True if reading is invalid (None or zero), False otherwise
        """
        return lux is None or lux == 0

    def reset(self) -> None:
        """Reset the light sensor by power cycling it.

        This performs a software reset by shutting down and re-enabling the sensor.
        """
        try:
            self._light_sensor.light_shutdown = True
            time.sleep(0.1)  # Allow time for sensor to power down
            self._light_sensor.light_shutdown = False
            self._log.debug("VEML6031X00 light sensor reset successfully")
        except Exception as e:
            self._log.error("Failed to reset VEML6031X00 light sensor:", e)
