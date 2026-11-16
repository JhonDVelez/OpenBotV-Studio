"""
Modulo que define el KinematicsWorker como hilo independiente.

Implementa la logica de control PID cartesiano sobre la base compartida
``ControlWorkerBase`` (src/services/robot/control_worker_base.py) y el
motor ``pid_engine``. El worker abre su propia conexion serial cuando hay
robot, ejecuta el lazo de control de forma autonoma y emite senales al GUI
para actualizacion de graficas y estado.

Plantas: el lazo del robot usa la telemetría del microcontrolador y el
lazo del gemelo la pose de PyBullet; cada uno se activa con su casilla
(``set_pid_targets``). Sin casillas el movimiento va por IK directa.

Flujo de ejecucion:
    1. Al activar modo cinematica con robot: HOME directo sin PID (2.5s).
       Sin robot: HOME virtual directo al gemelo + espera de llegada.
    2. Al recibir coordenadas del usuario:
       a. PID al HOME con compensaciones.
       b. PID al TARGET con compensaciones.
    3. Emite pid_iteration para la grafica CartesianPIDPlot.
    4. Emite status_changed para la barra de estado de la UI.
    5. Emite movement_finished al completar.
"""

import queue
import time

from src.features.kinematics.coordinate_correction import (
    apertura_de_garra,
    corregir_xy,
    corregir_z,
)
from src.services.data.utils.conversions import angulos_robotang
from src.services.robot.control_worker_base import (
    HOME_POS,
    LIMITES_HOME,
    LIMITES_TARGET,
    ControlWorkerBase,
)


class KinematicsWorker(ControlWorkerBase):
    """
    Worker independiente para control PID cartesiano en tiempo real.

    Abre su propia conexion serial (con robot) y ejecuta el lazo de
    control de forma autonoma, sin depender del RobotWorker ni del
    DataController. En modo virtual (sin robot) mueve solo el gemelo.
    """

    def __init__(self):
        super().__init__("KinematicsWorker", telemetry_init=[150.0] * 6)
        self.skip_home = False

    # ------------------------------------------------------------------ #
    #              EJECUCION DE SECUENCIA PID (en el hilo worker)          #
    # ------------------------------------------------------------------ #

    def _run_pid_sequence(self, tx, ty, tz):
        """Secuencia clásica: PID al HOME con compensaciones + PID al target.

        Args:
            tx, ty, tz (float): Objetivo cartesiano en mm.
        """
        self._pid_abort = False

        with self._claw_lock:
            angulo_garra = apertura_de_garra(self._claw_mm)

        tx_home, ty_home, tz_home = 185, 0, 170
        tz_home = corregir_z(tx_home, ty_home, tz_home)
        tx_home, ty_home = corregir_xy(tx_home, ty_home)

        t_start = time.time()

        self.status_changed.emit("PID: Moviendo a HOME...")
        ok_home = self._run_movement(
            [tx_home, ty_home, tz_home], LIMITES_HOME, angulo_garra,
            self._pid_robot, self._pid_sim, t_start=t_start)
        print("PID HOME convergido" if ok_home
              else "PID HOME: max iteraciones alcanzadas")

        if self._pid_abort:
            self._pid_abort = False
            return

        self.status_changed.emit("PID: Moviendo a target...")
        ok_target = self._run_movement(
            [tx, ty, tz], LIMITES_TARGET, angulo_garra,
            self._pid_robot, self._pid_sim, t_start=t_start)
        print("PID TARGET convergido" if ok_target
              else "PID TARGET: max iteraciones alcanzadas")

        self.status_changed.emit("Movimiento completado")
        self.movement_finished.emit()

    # ------------------------------------------------------------------ #
    #                  API PUBLICA (llamada desde el main thread)           #
    # ------------------------------------------------------------------ #

    def send_home_direct(self, com_port):
        """Arranca el hilo con serial y ejecuta el HOME directo."""
        self._com_port = com_port
        self._running = True
        self.start()

    def start_virtual(self, com_port=None):
        """Arranca el hilo en modo virtual (sin robot, solo gemelo)."""
        self._com_port = com_port
        self._running = True
        self._pid_abort = False
        self.start()

    def start_worker(self, com_port):
        self._com_port = com_port
        self._running = True
        self._pid_abort = False  # Asegurar reset PID
        self.start()

    def send_home_virtual(self):
        """Encola el HOME virtual: comando directo al gemelo y esperar."""
        self._work_queue.put(('send_home_virtual',))

    def execute_target(self, tx, ty, tz):
        self._work_queue.put(('execute_target', tx, ty, tz))

    def execute_pid_only(self, target_xyz, limites_deg, max_iter=800,
                         tolerancias=None, t_start=None, angulo_garra=None):
        self._work_queue.put(('execute_pid_only', target_xyz, limites_deg,
                              max_iter, tolerancias, t_start, angulo_garra))

    def send_home(self):
        self._work_queue.put(('send_home',))

    def stop_and_reset(self):
        """Detiene el movimiento, limpia el estado y regresa al HOME."""
        self.reset_state()
        self._last_commanded_angles = None
        with self._claw_lock:
            self._claw_mm = 30
            self._last_sent_claw_angle = -999
        self.send_home()

    # ------------------------------------------------------------------ #
    #                     MANTENIMIENTO FUERA DE PID                       #
    # ------------------------------------------------------------------ #

    def _mantener_garra(self):
        """Reenvía la apertura de garra al robot fuera del lazo PID."""
        if not self.gripper_control_enabled:
            return
        with self._claw_lock:
            angulo_garra_calculado = apertura_de_garra(self._claw_mm)
        if abs(angulo_garra_calculado - self._last_sent_claw_angle) <= 1.0:
            return
        fb = self._feedback_robot_deg()
        if fb is None:
            return
        fb[5] = angulo_garra_calculado
        self._last_commanded_angles = fb
        self._enviar_lazo_robot(fb)
        self._update_graph_with_angles(fb)
        self._last_sent_claw_angle = angulo_garra_calculado

    # ------------------------------------------------------------------ #
    #                  FLUJO PRINCIPAL (run del QThread)                   #
    # ------------------------------------------------------------------ #

    def run(self):
        con_serial = self._arrancar_telemetria()
        if not con_serial:
            if self._com_port:
                self.status_changed.emit("Error: No se pudo abrir serial")
                self.movement_finished.emit()
                return
            # Modo virtual (sin robot): continuar con el bucle de trabajo
            self.worker_ready.emit()

        if con_serial and not self.skip_home:
            self.status_changed.emit("Moviendo a HOME...")

            with self._claw_lock:
                angulo_garra = apertura_de_garra(self._claw_mm)

            home_pos = list(HOME_POS)
            home_pos[5] = angulo_garra

            # Comando directo a las plantas disponibles (robot y/o gemelo)
            self._despachar_directo(home_pos)
            self._update_graph_with_angles(home_pos)

            servo_positions = angulos_robotang(*home_pos)
            self._wait_for_position(servo_positions)

            self.input_enabled.emit()

        # Bucle de espera / trabajo
        while self._running:
            try:
                work = self._work_queue.get(timeout=0.01)
            except queue.Empty:
                self._mantener_garra()
                continue

            if work[0] == 'execute_target':
                _, tx, ty, tz = work
                self._run_pid_sequence(tx, ty, tz)
            elif work[0] == 'execute_pid_only':
                _, target, limites, max_iter, tol, t_start, angulo_garra = work
                if angulo_garra is None:
                    with self._claw_lock:
                        angulo_garra = apertura_de_garra(self._claw_mm)
                self._pid_abort = False
                self._run_movement(
                    target, limites, angulo_garra, self._pid_robot,
                    self._pid_sim, max_iter=max_iter, tolerancias=tol,
                    t_start=t_start)
            elif work[0] == 'send_home':
                home_pos = list(HOME_POS)
                self._despachar_directo(home_pos)
                self._update_graph_with_angles(home_pos)
                self._last_commanded_angles = list(home_pos)
                # Emitir señal para re-habilitar inputs tras el reinicio
                self.input_enabled.emit()
            elif work[0] == 'send_home_virtual':
                with self._claw_lock:
                    angulo_garra = apertura_de_garra(self._claw_mm)
                home_virtual = list(HOME_POS)
                home_virtual[5] = angulo_garra
                self._despachar_directo(home_virtual)
                self._update_graph_with_angles(home_virtual)
                self._last_commanded_angles = list(home_virtual)
                if self._wait_sim_reach(home_virtual):
                    self.status_changed.emit("HOME virtual alcanzado")
                self.input_enabled.emit()
            elif work[0] == 'stop':
                break

        self._telemetry_running = False
        self._close_serial()
