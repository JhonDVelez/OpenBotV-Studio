"""
Módulo que define el contador de fotogramas para control de cadencia.

Proporciona una instancia de módulo (``frame_counter``) que cuenta los
frames entrantes y emite una señal de procesamiento cada N frames,
permitiendo reducir la carga computacional del pipeline de visión.
"""

import threading
from PyQt6.QtCore import pyqtSignal, QObject
from src.services.data.signals import bus


class FrameCounter(QObject):
    """
    Contador de fotogramas con emisión periódica.

    Cuenta los frames entrantes y emite ``process_frame_signal`` cada
    ``_interval`` ticks, permitiendo espaciar el procesamiento pesado
    de visión artificial.

    Signals:
        process_frame_signal: Se emite cuando se alcanza el intervalo.
    """
    process_frame_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._interval = bus.cfg_get_param(
            "settings.json", "camera", "view", "interval", default=4)
        self._counter = 0
        self._lock = threading.Lock()

    def tick(self):
        """
        Incrementa el contador y emite la señal al alcanzar el intervalo.
        """
        with self._lock:
            self._counter += 1
            if self._counter >= self._interval:
                try:
                    self.process_frame_signal.emit()
                finally:
                    self._counter = 0

    def reset(self):
        """
        Reinicia el contador a cero.
        """
        with self._lock:
            self._counter = 0

    def set_interval(self, interval: int):
        """
        Establece un nuevo intervalo de emisión y persiste el cambio.

        Args:
            interval (int): Nuevo intervalo en frames.
        """
        with self._lock:
            self._interval = int(interval)
            self._counter = 0
            bus.cfg_request_change("settings.json", ["camera", "view",
                                                "interval"], self._interval)


# Instancia única del contador para toda la aplicación.
frame_counter = FrameCounter()
