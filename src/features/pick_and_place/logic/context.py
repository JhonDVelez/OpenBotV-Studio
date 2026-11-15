"""
Módulo que define el contexto de datos para la secuencia Pick and Place.

Dataclass que encapsula todo el estado compartido de la operación,
evitando el acoplamiento fuerte a través de la herencia en el worker.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PickPlaceContext:
    """Contenedor de estado para la operación de Pick and Place."""

    # Estado de la operación
    selected_color: Optional[str] = None
    tamano_seleccionado: int = 30
    sphere_poses: dict = field(default_factory=dict)
    place_target_coords: Optional[dict] = None
    ik_target: Optional[list] = None

    # Estado de la pinza
    gripper_open: float = -112.0
    gripper_closed: float = 7.0
    gripper_max_closed: float = 21.0

    # Control de movimiento y feedback
    last_positions: Optional[list] = None
    current_feedback: object = None
    current_target: Optional[list] = None

    def reset(self) -> None:
        """Limpia el estado interno para una nueva operación."""
        self.selected_color = None
        self.tamano_seleccionado = 30
        self.place_target_coords = None
        self.ik_target = None
        self.current_target = None
        self.last_positions = None
        self.current_feedback = None
        # Mantenemos sphere_poses y configuración de pinza
