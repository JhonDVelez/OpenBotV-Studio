"""Tests de integración de la base de workers de control (dual planta).

Cierra el lazo real worker -> bus -> planta falsa del gemelo -> feedback:
    worker (_enviar_lazo_sim) -> bus.sim_update_pybullet_signal
    -> GemeloFalso.update_target -> integración a velocidad limitada
    -> bus.sim_model_position_signal -> worker (_on_sim_model_positions)
    -> lazo PID / IK directa.

Contratos verificados:
    - Con la casilla del gemelo activa, el lazo PID del gemelo converge.
    - Con ambas casillas apagadas, va una solución de IK directa única y
      el movimiento termina cuando el feedback del gemelo se estabiliza.
    - sync_pid_targets deshabilita y desmarca la casilla del robot sin
      robot conectado.
"""

import numpy as np
import pytest

from src.services.data.signals import bus
from src.services.data.utils import deg_to_rad
from src.services.robot.control_worker_base import HOME_POS, LIMITES_TARGET
from src.services.robot.pid_engine import cinematica_directa
from src.features.kinematics.kinematics_worker import KinematicsWorker
from src.features.kinematics.kinematics_widget import KinematicsWidget

RATE_DEG_S = 68.8  # 1.2 rad/s = velocidad máxima de PyBullet
PASO_MS = 50


class GemeloFalso:
    """Planta falsa del gemelo: integra hacia el último comando PyBullet."""

    def __init__(self):
        self.q_deg = list(HOME_POS)   # grados canónicos
        self.destino_deg = list(HOME_POS)

    def update_target(self, rad_list):
        """Slot de bus.sim_update_pybullet_signal (radianes canónicos).

        La señal viaja en espacio canónico; el volteo a espacio PyBullet
        ocurre solo al emitir la pose de readback (índices 4 y 5).
        """
        self.destino_deg = [float(np.degrees(v)) for v in rad_list]

    def paso(self):
        """Integra un paso y emite la pose en radianes PyBullet."""
        delta = np.array(self.destino_deg) - np.array(self.q_deg)
        paso_arr = np.clip(delta, -RATE_DEG_S * PASO_MS / 1000.0,
                           RATE_DEG_S * PASO_MS / 1000.0)
        self.q_deg = list(np.array(self.q_deg) + paso_arr)
        rad = np.radians(self.q_deg)
        # Emitir en espacio PyBullet: índices 4 y 5 negados
        bus.sim_model_position_signal.emit(
            [float(rad[0]), float(rad[1]), float(rad[2]), float(rad[3]),
             float(-rad[4]), float(-rad[5])], {})


def fk_deg(q_deg):
    """FK desde grados canónicos."""
    return cinematica_directa(
        np.radians([q_deg[0], q_deg[1], q_deg[2], q_deg[4]])).tolist()


class TestControlWorkerBase:
    """Integración de la base de workers con el gemelo falso."""

    @pytest.fixture()
    def gemelo(self, qtbot):
        """Gemelo falso conectado al bus con temporizador de integración."""
        from PyQt6.QtCore import QTimer
        planta = GemeloFalso()
        bus.sim_update_pybullet_signal.connect(planta.update_target)
        bus.sim_model_position_signal.emit(
            [float(np.radians(v)) for v in
             [planta.q_deg[0], planta.q_deg[1], planta.q_deg[2],
              planta.q_deg[3], -planta.q_deg[4], -planta.q_deg[5]]], {})
        timer = QTimer()
        timer.timeout.connect(planta.paso)
        timer.start(PASO_MS)
        bus.sim_set_running(True)
        yield planta
        timer.stop()
        bus.sim_update_pybullet_signal.disconnect(planta.update_target)
        bus.sim_set_running(False)

    @pytest.fixture()
    def worker(self, qtbot):
        """KinematicsWorker virtual limpio tras el test."""
        w = KinematicsWorker()
        yield w
        w.force_abort()

    def test_lazo_sim_converge(self, qtbot, gemelo, worker):
        worker.set_pid_targets(False, True)
        worker.start_virtual()

        q_objetivo = [10.0, -40.0, 110.0, 0, 30.0, 0.0]
        target_xyz = fk_deg(q_objetivo)
        with qtbot.waitSignal(worker.movement_finished, timeout=90000):
            worker.execute_pid_only(target_xyz, LIMITES_TARGET,
                                    angulo_garra=0.0)

        error = np.abs(np.array(fk_deg(gemelo.q_deg)) - np.array(target_xyz))
        assert error[0] < 5.0 and error[1] < 7.0 and error[2] < 5.0

    def test_sin_casillas_envia_ik_directa(self, qtbot, gemelo, worker):
        worker.set_pid_targets(False, False)
        worker.start_virtual()

        q_objetivo = [10.0, -40.0, 110.0, 0, 30.0, 0.0]
        target_xyz = fk_deg(q_objetivo)
        with qtbot.waitSignal(worker.movement_finished, timeout=90000):
            worker.execute_pid_only(target_xyz, LIMITES_TARGET,
                                    angulo_garra=0.0)

        error = np.abs(np.array(fk_deg(gemelo.q_deg)) - np.array(target_xyz))
        assert error[0] < 5.0 and error[1] < 7.0 and error[2] < 5.0


class TestSyncPidTargets:
    """Sincronización de casillas del widget según conexión del robot."""

    def test_sin_robot_deshabilita_casilla_robot(self, qapp):
        widget = KinematicsWidget()
        widget.chk_pid_robot.setChecked(True)
        widget.chk_pid_sim.setChecked(True)

        widget.sync_pid_targets(False)

        assert not widget.chk_pid_robot.isEnabled()
        assert not widget.chk_pid_robot.isChecked()
        assert widget.chk_pid_sim.isEnabled()
        assert widget.chk_pid_sim.isChecked()

    def test_con_robot_mantiene_casillas(self, qapp):
        widget = KinematicsWidget()
        widget.chk_pid_robot.setChecked(True)

        widget.sync_pid_targets(True)

        assert widget.chk_pid_robot.isEnabled()
        assert widget.chk_pid_robot.isChecked()

    def test_get_set_pid_targets(self, qapp):
        widget = KinematicsWidget()
        widget.set_pid_targets(robot=False, sim=True)
        assert widget.get_pid_targets() == {"robot": False, "sim": True}
