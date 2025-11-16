"""This module defines the `DRV2605LManager` class, which provides a high-level interface
for controlling the TI DRV2605L haptic motor driver as a magnetorquer controller.

The DRV2605L is repurposed to drive magnetorquer coils for satellite attitude control.
It provides PWM-based current control which translates to magnetic dipole moments.

**Hardware Configuration:**
- I2C Address: 0x5A (default)
- Each device controls one axis of magnetic torque

**Usage:**
```python
logger = Logger()
i2c = busio.I2C(board.SCL, board.SDA)
magnetorquer_x = DRV2605LManager(logger, i2c, address=0x5A, axis='x')
magnetorquer_x.set_dipole_moment(0.0, 0.0, 0.5)  # Set Z-axis dipole
```
"""

try:
    from typing import Tuple
except ImportError:
    pass

from adafruit_drv2605 import DRV2605
from busio import I2C

from ....logger import Logger
from ....protos.magnetorquer import MagnetorquerProto
from ...exception import HardwareInitializationError


class DRV2605LManager(MagnetorquerProto):
    """Manages the TI DRV2605L haptic driver as a magnetorquer controller.
    
    This class interfaces with the DRV2605L to control magnetorquer coils for
    satellite attitude control. The driver's PWM output is used to generate
    controlled magnetic fields.
    """

    # Constants for converting dipole moment to PWM values
    # These should be calibrated based on coil properties and desired dipole range
    MAX_DIPOLE_MOMENT = 1.0  # Maximum dipole moment in A·m²
    PWM_MAX = 127  # Maximum PWM value for DRV2605L in ERM mode

    def __init__(
        self,
        logger: Logger,
        i2c: I2C,
        address: int = 0x5A,
        axis: str = "x",
        max_dipole: float = 1.0,
    ) -> None:
        """Initializes the DRV2605LManager.

        Args:
            logger: The logger to use.
            i2c: The I2C bus connected to the chip.
            address: The I2C address of the DRV2605L (default 0x5A).
            axis: The axis this magnetorquer controls ('x', 'y', or 'z').
            max_dipole: Maximum dipole moment in A·m² for this magnetorquer.

        Raises:
            HardwareInitializationError: If the magnetorquer fails to initialize.
        """
        self._log: Logger = logger
        self._axis = axis.lower()
        self._max_dipole = max_dipole
        self._current_dipole = 0.0

        if self._axis not in ["x", "y", "z"]:
            raise ValueError(f"Invalid axis '{axis}'. Must be 'x', 'y', or 'z'.")

        try:
            self._log.debug(f"Initializing DRV2605L magnetorquer on axis {self._axis}")
            self._driver: DRV2605 = DRV2605(i2c, address=address)
            
            # Configure for direct PWM control (ERM mode)
            self._driver.use_ERM()
            
            # Set to standby initially
            self._current_dipole = 0.0
            self._set_pwm(0)
            
        except Exception as e:
            raise HardwareInitializationError(
                f"Failed to initialize DRV2605L magnetorquer on axis {self._axis}"
            ) from e

    def _dipole_to_pwm(self, dipole: float) -> int:
        """Converts dipole moment to PWM value.

        Args:
            dipole: Dipole moment in A·m²

        Returns:
            PWM value (0-127)
        """
        # Clamp dipole to valid range
        dipole = max(-self._max_dipole, min(self._max_dipole, dipole))
        
        # Convert to PWM (0-127 range)
        # Note: DRV2605L doesn't support negative values, so we use absolute value
        pwm = int(abs(dipole) / self._max_dipole * self.PWM_MAX)
        return max(0, min(self.PWM_MAX, pwm))

    def _set_pwm(self, pwm_value: int) -> None:
        """Sets the PWM value on the DRV2605L.

        Args:
            pwm_value: PWM value (0-127)
        """
        # Set real-time playback mode and write PWM value
        self._driver.mode = 0x05  # Real-time playback mode
        self._driver.realtime_value = pwm_value

    def set_dipole_moment(self, x: float, y: float, z: float) -> None:
        """Sets the magnetic dipole moment for the magnetorquer.

        Only the component matching this controller's axis will be applied.

        Args:
            x: The x-axis dipole moment in ampere-meters squared (A·m²)
            y: The y-axis dipole moment in ampere-meters squared (A·m²)
            z: The z-axis dipole moment in ampere-meters squared (A·m²)

        Raises:
            RuntimeError: If the dipole moment cannot be set due to hardware issues
        """
        try:
            # Select the appropriate dipole component for this axis
            dipole_map = {"x": x, "y": y, "z": z}
            target_dipole = dipole_map[self._axis]
            
            # Convert to PWM and apply
            pwm = self._dipole_to_pwm(target_dipole)
            self._set_pwm(pwm)
            self._current_dipole = target_dipole
            
            self._log.debug(
                f"Set magnetorquer {self._axis} dipole to {target_dipole:.3f} A·m² (PWM: {pwm})"
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to set dipole moment on magnetorquer {self._axis}"
            ) from e

    def get_dipole_moment(self) -> Tuple[float, float, float]:
        """Gets the current magnetic dipole moment from the magnetorquer.

        Returns a tuple with the dipole on this controller's axis and zero on others.

        Returns:
            A tuple containing the x, y, and z dipole moments in ampere-meters squared (A·m²)

        Raises:
            RuntimeError: If the dipole moment cannot be read due to hardware issues
        """
        try:
            # Build tuple with current dipole on appropriate axis
            if self._axis == "x":
                return (self._current_dipole, 0.0, 0.0)
            elif self._axis == "y":
                return (0.0, self._current_dipole, 0.0)
            else:  # z
                return (0.0, 0.0, self._current_dipole)
        except Exception as e:
            raise RuntimeError(
                f"Failed to read dipole moment from magnetorquer {self._axis}"
            ) from e

    def disable(self) -> None:
        """Disables the magnetorquer, setting all dipole moments to zero.

        Raises:
            RuntimeError: If the magnetorquer cannot be disabled due to hardware issues
        """
        try:
            self._set_pwm(0)
            self._current_dipole = 0.0
            self._log.debug(f"Disabled magnetorquer {self._axis}")
        except Exception as e:
            raise RuntimeError(
                f"Failed to disable magnetorquer {self._axis}"
            ) from e
