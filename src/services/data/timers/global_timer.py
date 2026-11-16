"""
Módulo que define el temporizador global de sincronización.

Proporciona una instancia de módulo (``global_timer``) de QTimer que
emite ticks a diferentes frecuencias para la actualización del modelo
3D, los gráficos y la sincronización con el robot físico.

El timer NO arranca automáticamente al importar: el primer consumidor
(de forma habitual ``DataController``) debe llamar ``global_timer.start()``
(comportamiento idéntico al singleton anterior, donde ``get_instance()``
lo arrancaba).
"""

from PyQt6.QtCore import pyqtSignal, QObject, QTimer


class GlobalTimer(QObject):
    """
    Temporizador global que coordina los ciclos de actualización del sistema.

    Emite señales a distintas frecuencias:
    - model_tick: Alta frecuencia para actualización suave del modelo 3D.
    - update_tick: Frecuencia media para gráficos.
    - sync_simulation_tick / sync_robot_tick: Baja frecuencia para sincronización.

    Signals:
        sync_simulation_tick: Pulso de sincronización de simulación.
        update_tick: Actualización de gráficos.
        model_tick: Actualización del modelo 3D.
        sync_robot_tick: Pulso de sincronización del robot físico.
    """
    sync_simulation_tick = pyqtSignal()
    update_tick = pyqtSignal()
    model_tick = pyqtSignal()
    sync_robot_tick = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._timer = QTimer()
        self._timer.setInterval(4)
        self._timer.timeout.connect(self._tick)

        self._sync_counter = 0
        self._model_counter = 0

    def _tick(self):
        """
        Ejecuta la lógica de distribución de ticks según contadores internos.
        """
        self._sync_counter += 1
        self._model_counter += 1

        if self._model_counter >= 4:
            self.model_tick.emit()
            self._model_counter = 0

        if self._sync_counter >= 25:
            self.sync_robot_tick.emit()
            self.sync_simulation_tick.emit()
            self._sync_counter = 0
        else:
            self.update_tick.emit()

    def start(self):
        """
        Inicia el temporizador si no está activo (idempotente).
        """
        if not self._timer.isActive():
            self._timer.start()

    def stop(self):
        """
        Detiene el temporizador si está activo.
        """
        if self._timer.isActive():
            self._timer.stop()


# Instancia única del temporizador para toda la aplicación.
global_timer = GlobalTimer()
