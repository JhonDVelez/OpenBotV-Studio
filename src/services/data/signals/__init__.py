"""
Bus de señales del sistema.

Un único módulo define el bus: ``signals.bus`` con la clase ``SignalBus`` y
la instancia compartida ``bus``. Todas las señales del proceso viven ahí,
con prefijo de dominio (sim_, phy_, kin_, sld_, cam_, pp_, cfg_, search_,
theme_). Los antiguos gestores por dominio (los 12 SignalManager) fueron
eliminados; sus equivalentes en el bus están documentados en el docstring
de ``bus.py``.

Uso:
    from src.services.data.signals import bus
    bus.sim_update_robot_signal.emit(posiciones)
    bus.cfg_request_change("settings.json", ["camera", "view"], valor)
"""

from .bus import bus
from .draw_view import DrawViewSignalManager

__all__ = [
    'bus',
    'DrawViewSignalManager',
]
