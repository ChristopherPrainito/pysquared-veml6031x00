"""Magnetorquer dipole moment sensor reading."""

try:
    from typing import Tuple
except ImportError:
    pass

from .base import Reading


class DipoleMoment(Reading):
    """Magnetorquer dipole moment reading in ampere-meters squared (A·m²)."""

    def __init__(self, x: float, y: float, z: float) -> None:
        """Initialize the dipole moment sensor reading.

        Args:
            x: The x-axis dipole moment in ampere-meters squared (A·m²)
            y: The y-axis dipole moment in ampere-meters squared (A·m²)
            z: The z-axis dipole moment in ampere-meters squared (A·m²)
        """
        super().__init__()
        self.x = x
        self.y = y
        self.z = z

    @property
    def value(self) -> Tuple[float, float, float]:
        """Dipole moment in x, y, z ampere-meters squared (A·m²).

        Returns:
            A tuple containing the x, y, and z components of the dipole moment.
        """
        return (self.x, self.y, self.z)
