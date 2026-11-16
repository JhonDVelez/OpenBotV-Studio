"""
Módulo que define el gestor de estado de la vista de dibujo.

Controla la visibilidad de los overlays (cuadrícula ChArUco y
geometrías de esferas) que se dibujan sobre el frame de cámara.
"""

import threading
from .bus import bus


class DrawViewSignalManager:
    """
    Gestor del estado de visualización de overlays en la cámara.

    Permite al usuario alternar la visualización de la cuadrícula
    ChArUco y las esferas detectadas sobre el feed de video.

    Usa la instancia de módulo ``draw_view`` exportada al final de
    este módulo (antes era un singleton con get_instance()).
    """
    _charuco = False
    _circle = False

    def _init_state(self):
        """
        Inicializa el estado desde la configuración persistente.
        """
        self._lock = threading.Lock()
        state = bus.cfg_get_param(
            "settings.json", "camera", "view", default={})
        self._charuco = state.get("charuco", False)
        self._circle = state.get("circle", False)

    def _init_state(self):
        """
        Inicializa el estado desde la configuración persistente.
        """
        self._lock = threading.Lock()
        state = bus.cfg_get_param(
            "settings.json", "camera", "view", default={})
        self._charuco = state.get("charuco", False)
        self._circle = state.get("circle", False)

    def set_charuco(self, checked: bool):
        """
        Activa o desactiva el overlay de cuadrícula ChArUco.

        Args:
            checked (bool): True para mostrar la cuadrícula.
        """
        with self._lock:
            self._charuco = checked
            bus.cfg_request_change("settings.json", ["camera",
                                      "view", "charuco"], checked)

    def set_circle(self, checked: bool):
        """
        Activa o desactiva el overlay de geometría de las esferas.

        Args:
            checked (bool): True para mostrar las esferas.
        """
        with self._lock:
            self._circle = checked
            bus.cfg_request_change("settings.json", ["camera",
                                      "view", "circle"], checked)

    def get_state(self):
        """
        Obtiene el estado actual de ambos overlays.

        Returns:
            tuple: (charuco_visible, circle_visible).
        """
        with self._lock:
            return self._charuco, self._circle


# Instancia única del gestor de estado de dibujo para toda la aplicación.
draw_view = DrawViewSignalManager()
draw_view._init_state()
