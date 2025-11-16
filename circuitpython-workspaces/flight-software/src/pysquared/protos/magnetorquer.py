"""This protocol specifies the interface that any magnetorquer implementation must
adhere to, ensuring consistent behavior across different magnetorquer hardware.
"""

try:
    from typing import Tuple
except ImportError:
    pass


class MagnetorquerProto:
    """Protocol defining the interface for a Magnetorquer.
    
    Magnetorquers use magnetic torque to control satellite attitude by interacting
    with Earth's magnetic field. This protocol provides methods to set dipole moments
    and read current magnetorquer states.
    """

    def set_dipole_moment(self, x: float, y: float, z: float) -> None:
        """Sets the magnetic dipole moment for the magnetorquer.

        Args:
            x: The x-axis dipole moment in ampere-meters squared (A·m²)
            y: The y-axis dipole moment in ampere-meters squared (A·m²)
            z: The z-axis dipole moment in ampere-meters squared (A·m²)

        Raises:
            RuntimeError: If the dipole moment cannot be set due to hardware issues
        """
        ...

    def get_dipole_moment(self) -> Tuple[float, float, float]:
        """Gets the current magnetic dipole moment from the magnetorquer.

        Returns:
            A tuple containing the x, y, and z dipole moments in ampere-meters squared (A·m²)

        Raises:
            RuntimeError: If the dipole moment cannot be read due to hardware issues
        """
        ...

    def disable(self) -> None:
        """Disables the magnetorquer, setting all dipole moments to zero.

        Raises:
            RuntimeError: If the magnetorquer cannot be disabled due to hardware issues
        """
        ...
