"""Unit tests for the DRV2605LManager class.

This module contains unit tests for the `DRV2605LManager` class, which manages
the TI DRV2605L haptic motor driver as a magnetorquer controller. The tests cover
initialization, dipole moment setting, retrieval, and error handling.
"""

from typing import Generator
from unittest.mock import MagicMock, patch

import pytest
from pysquared.hardware.exception import HardwareInitializationError
from pysquared.hardware.magnetorquer.manager.drv2605l import DRV2605LManager


@pytest.fixture
def mock_i2c():
    """Fixture for mock I2C bus."""
    return MagicMock()


@pytest.fixture
def mock_logger():
    """Fixture for mock Logger."""
    return MagicMock()


@pytest.fixture
def mock_drv2605(mock_i2c: MagicMock) -> Generator[MagicMock, None, None]:
    """Mocks the DRV2605 class.

    Args:
        mock_i2c: Mocked I2C bus.

    Yields:
        A MagicMock instance of DRV2605.
    """
    with patch(
        "pysquared.hardware.magnetorquer.manager.drv2605l.DRV2605"
    ) as mock_class:
        mock_instance = MagicMock()
        mock_instance.mode = 0x00
        mock_instance.realtime_value = 0
        mock_class.return_value = mock_instance
        yield mock_class


def test_create_magnetorquer_x_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests successful creation of a DRV2605L magnetorquer on X axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, address=0x5A, axis="x")

    assert magnetorquer._axis == "x"
    assert magnetorquer._max_dipole == 1.0
    assert magnetorquer._current_dipole == 0.0
    mock_drv2605.assert_called_once_with(mock_i2c, address=0x5A)
    mock_drv2605.return_value.use_ERM.assert_called_once()
    mock_logger.debug.assert_called_with("Initializing DRV2605L magnetorquer on axis x")


def test_create_magnetorquer_y_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests successful creation of a DRV2605L magnetorquer on Y axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="y", max_dipole=2.0)

    assert magnetorquer._axis == "y"
    assert magnetorquer._max_dipole == 2.0


def test_create_magnetorquer_z_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests successful creation of a DRV2605L magnetorquer on Z axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="Z")

    assert magnetorquer._axis == "z"


def test_create_magnetorquer_invalid_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests that invalid axis raises ValueError.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    with pytest.raises(ValueError) as excinfo:
        DRV2605LManager(mock_logger, mock_i2c, axis="invalid")

    assert "Invalid axis" in str(excinfo.value)


def test_create_magnetorquer_failed(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests that initialization failure raises HardwareInitializationError.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    mock_drv2605.side_effect = Exception("Simulated DRV2605 failure")

    with pytest.raises(HardwareInitializationError) as excinfo:
        DRV2605LManager(mock_logger, mock_i2c, axis="x")

    assert "Failed to initialize DRV2605L magnetorquer on axis x" in str(excinfo.value)


def test_set_dipole_moment_x_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests setting dipole moment on X axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x", max_dipole=1.0)
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(0.5, 0.0, 0.0)

    # PWM should be 50% of max (127) = 63.5 -> 63
    assert mock_driver.mode == 0x05
    assert mock_driver.realtime_value == 63
    assert magnetorquer._current_dipole == 0.5


def test_set_dipole_moment_y_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests setting dipole moment on Y axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="y", max_dipole=1.0)
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(0.0, 0.8, 0.0)

    # PWM should be 80% of max (127) = 101.6 -> 101
    assert mock_driver.realtime_value == 101
    assert magnetorquer._current_dipole == 0.8


def test_set_dipole_moment_z_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests setting dipole moment on Z axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="z", max_dipole=2.0)
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(0.0, 0.0, 1.0)

    # PWM should be 50% of max (127) = 63.5 -> 63
    assert mock_driver.realtime_value == 63
    assert magnetorquer._current_dipole == 1.0


def test_set_dipole_moment_clamping_positive(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests that dipole moment is clamped to max value.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x", max_dipole=1.0)
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(5.0, 0.0, 0.0)

    # Should clamp to max_dipole (1.0), which gives PWM 127
    assert mock_driver.realtime_value == 127
    assert magnetorquer._current_dipole == 5.0  # Stores requested value


def test_set_dipole_moment_clamping_negative(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests that negative dipole moment is handled (absolute value).

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x", max_dipole=1.0)
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(-0.5, 0.0, 0.0)

    # Should use absolute value: 0.5 -> PWM 63
    assert mock_driver.realtime_value == 63
    assert magnetorquer._current_dipole == -0.5


def test_set_dipole_moment_zero(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests setting dipole moment to zero.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x")
    mock_driver = mock_drv2605.return_value

    magnetorquer.set_dipole_moment(0.0, 0.0, 0.0)

    assert mock_driver.realtime_value == 0
    assert magnetorquer._current_dipole == 0.0


def test_set_dipole_moment_error(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests error handling when setting dipole moment fails.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x")
    mock_driver = mock_drv2605.return_value

    # Simulate error when setting mode
    type(mock_driver).mode = property(
        lambda self: 0,
        lambda self, value: (_ for _ in ()).throw(Exception("Hardware error")),
    )

    with pytest.raises(RuntimeError) as excinfo:
        magnetorquer.set_dipole_moment(0.5, 0.0, 0.0)

    assert "Failed to set dipole moment on magnetorquer x" in str(excinfo.value)


def test_get_dipole_moment_x_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests getting dipole moment from X axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x")
    magnetorquer._current_dipole = 0.7

    dipole = magnetorquer.get_dipole_moment()

    assert dipole == (0.7, 0.0, 0.0)


def test_get_dipole_moment_y_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests getting dipole moment from Y axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="y")
    magnetorquer._current_dipole = 0.3

    dipole = magnetorquer.get_dipole_moment()

    assert dipole == (0.0, 0.3, 0.0)


def test_get_dipole_moment_z_axis(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests getting dipole moment from Z axis.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="z")
    magnetorquer._current_dipole = -0.5

    dipole = magnetorquer.get_dipole_moment()

    assert dipole == (0.0, 0.0, -0.5)


def test_disable_magnetorquer(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests disabling the magnetorquer.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x")
    mock_driver = mock_drv2605.return_value
    magnetorquer._current_dipole = 0.5

    magnetorquer.disable()

    assert mock_driver.realtime_value == 0
    assert magnetorquer._current_dipole == 0.0


def test_disable_magnetorquer_error(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests error handling when disabling magnetorquer fails.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x")
    mock_driver = mock_drv2605.return_value

    # Simulate error when setting realtime_value
    type(mock_driver).realtime_value = property(
        lambda self: 0,
        lambda self, value: (_ for _ in ()).throw(Exception("Hardware error")),
    )

    with pytest.raises(RuntimeError) as excinfo:
        magnetorquer.disable()

    assert "Failed to disable magnetorquer x" in str(excinfo.value)


def test_dipole_to_pwm_conversion(
    mock_drv2605: MagicMock,
    mock_i2c: MagicMock,
    mock_logger: MagicMock,
) -> None:
    """Tests the dipole to PWM conversion function.

    Args:
        mock_drv2605: Mocked DRV2605 class.
        mock_i2c: Mocked I2C bus.
        mock_logger: Mocked Logger instance.
    """
    magnetorquer = DRV2605LManager(mock_logger, mock_i2c, axis="x", max_dipole=1.0)

    # Test various conversions
    assert magnetorquer._dipole_to_pwm(0.0) == 0
    assert magnetorquer._dipole_to_pwm(1.0) == 127
    assert magnetorquer._dipole_to_pwm(0.5) == 63
    assert magnetorquer._dipole_to_pwm(-0.5) == 63  # Absolute value
    assert magnetorquer._dipole_to_pwm(2.0) == 127  # Clamped to max
    assert magnetorquer._dipole_to_pwm(-2.0) == 127  # Clamped to max (abs)
