"""
Paquete de temporización global de la aplicación.

Expone el temporizador de sincronización (``global_timer``), el
contador de fotogramas (``frame_counter``) para la sincronización de
procesamiento de visión, y sus clases subyacentes.
"""

from .global_timer import GlobalTimer, global_timer
from .frame_counter import FrameCounter, frame_counter

__all__ = ['GlobalTimer', 'global_timer', 'FrameCounter', 'frame_counter']
