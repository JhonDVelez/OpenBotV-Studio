"""
Utilidades de conversión servo para los ejecutores de Pick and Place.

Funciones planas compartidas por PickExecutor y PlaceExecutor.
"""


def relative_to_servo(positions):
    """Convierte ángulos relativos a posiciones absolutas de servo."""
    return [float(value + 150.0) for value in positions]


def with_gripper(status, gripper_degrees):
    """Copia un objetivo y establece la pinza en grados relativos."""
    updated = list(status)
    updated[-1] = float(gripper_degrees + 150.0)
    return updated
