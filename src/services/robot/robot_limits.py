"""
Utilidades de conversión y limitación para comandos del robot físico.

Mantiene las utilidades compartidas entre KinematicsWorker y el servicio
serial: saturación de límites físicos articulares. Las conversiones
angulo-servo (angulos_robotang / robotang_angulos) viven en
``src.services.data.utils.conversions``.
"""

import math
import numpy as np

_LIMITS_DEG = [(-100.0, 100.0), (-90.0, 90.0),
               (-130.0, 130.0), (-90.0, 120.0)]


def apply_physical_limits(q_rad, limites_deg=None):
    """Satura cada articulación según los límites físicos del robot.

    Args:
        q_rad: Ángulos articulares en radianes (iterable de 4 valores).
        limites_deg: Límites (min, max) por articulación en grados.
            Si es None se usan los límites por defecto del robot.

    Returns:
        np.ndarray: Ángulos saturados en radianes.
    """
    if limites_deg is None:
        limites_deg = _LIMITS_DEG
    q_ajustado = []
    for index, angle in enumerate(q_rad):
        angle_deg = math.degrees(angle)
        low, high = limites_deg[index]
        q_ajustado.append(math.radians(max(low, min(angle_deg, high))))
    return np.array(q_ajustado)
