"""Unit tests for the DipoleMoment sensor reading class.

This module contains unit tests for the `DipoleMoment` class, which represents
magnetorquer dipole moment readings in ampere-meters squared (A·m²).
"""

import time
from unittest.mock import patch

from pysquared.sensor_reading.dipole_moment import DipoleMoment


def test_dipole_moment_initialization() -> None:
    """Tests successful creation of a DipoleMoment instance."""
    dipole = DipoleMoment(x=0.5, y=-0.3, z=0.8)

    assert dipole.x == 0.5
    assert dipole.y == -0.3
    assert dipole.z == 0.8


def test_dipole_moment_value_property() -> None:
    """Tests the value property returns correct tuple."""
    dipole = DipoleMoment(x=1.0, y=2.0, z=3.0)

    value = dipole.value
    assert isinstance(value, tuple)
    assert len(value) == 3
    assert value == (1.0, 2.0, 3.0)


def test_dipole_moment_timestamp() -> None:
    """Tests that timestamp is set correctly."""
    with patch("time.time", return_value=1234567890.0):
        dipole = DipoleMoment(x=0.1, y=0.2, z=0.3)
        assert dipole.timestamp == 1234567890.0


def test_dipole_moment_to_dict() -> None:
    """Tests conversion to dictionary for JSON serialization."""
    with patch("time.time", return_value=1234567890.0):
        dipole = DipoleMoment(x=0.5, y=-0.3, z=0.8)
        result = dipole.to_dict()

        assert isinstance(result, dict)
        assert result["timestamp"] == 1234567890.0
        assert result["value"] == (0.5, -0.3, 0.8)


def test_dipole_moment_zero_values() -> None:
    """Tests DipoleMoment with all zero values."""
    dipole = DipoleMoment(x=0.0, y=0.0, z=0.0)

    assert dipole.x == 0.0
    assert dipole.y == 0.0
    assert dipole.z == 0.0
    assert dipole.value == (0.0, 0.0, 0.0)


def test_dipole_moment_negative_values() -> None:
    """Tests DipoleMoment with negative values."""
    dipole = DipoleMoment(x=-1.5, y=-2.5, z=-3.5)

    assert dipole.x == -1.5
    assert dipole.y == -2.5
    assert dipole.z == -3.5
    assert dipole.value == (-1.5, -2.5, -3.5)
