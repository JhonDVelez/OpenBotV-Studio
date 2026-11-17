"""
Clase base para los workers de control (cinemática y Pick & Place).

Centraliza la infraestructura que ambos workers compartían duplicada:
conexión serial propia, hilo de telemetría, buffer de feedback del gemelo
(PyBullet), sinks de envío y el orquestador de movimiento multi-planta
(`run_movement` del motor PID compartido).

Modelo de datos de la clase (port de los workers originales):
    - Espacios de coordenadas: grados canónicos (usuario) [q1, q2, q3, 0,
      q4, garra] y espacio servo 0-300 (conversiones de
      ``src.services.data.utils.conversions``).
    - Casillas PID por planta: ``set_pid_targets(robot, sim)`` decide qué
      plantas corren lazo cerrado; el resto recibe una única solución de
      IK directa al inicio del movimiento.
    - Planta de avance: decide el fin del movimiento (telemetría del
      robot si hay serial, si no la pose de PyBullet).

Conexiones:
    - ``bus.sim_model_position_signal`` -> buffer de feedback del gemelo.
    - ``bus.sim_update_pybullet_signal`` <- sink del lazo del gemelo.
    - ``serial_ports`` <- IO serial propia del worker (owner_id).
"""

import queue
import re
import serial
import threading
import time

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot

from src.services.data.signals import bus
from src.services.data.utils import deg_to_rad, rad_to_deg
from src.services.data.utils.conversions import angulos_robotang, robotang_angulos
from src.services.robot.pid_engine import (
    LINKS_DEF,
    TS_DEF,
    PlantPID,
    LoopGate,
    cinematica_directa,
    run_movement,
)
from src.services.robot.serial_manager import serial_ports

# Pose de HOME en grados canónicos [q1, q2, q3, 0, q4, garra].
# La garra (índice 5) se resuelve dinámicamente según el tamaño objetivo.
HOME_POS = [0, -45, 120, 0, 30, 0]
LIMITES_HOME = [(-10, 10), (-50, -40), (0, 130), (0, 120)]
LIMITES_TARGET = [(-100, 100), (-90, 90), (-130, 130), (-90, 120)]


class ControlWorkerBase(QThread):
    """Base de los workers de control con plantas robot y gemelo.

    Args:
        owner_id (str): Identificador de dueño del puerto serial
            ("KinematicsWorker" / "PickAndPlaceWorker").
        telemetry_init (list): Estado inicial de la telemetría; los
            workers sin serial arrancan con ``[None] * 6`` y los que
            asumen servo centrado con ``[150.0] * 6``.

    Señales compartidas (emitidas desde el hilo worker, procesadas en el
    hilo principal):
        pid_iteration(t_relativo, pos_real_xyz, pos_target_xyz):
            Para la gráfica CartesianPIDPlot.
        status_changed(mensaje): Para la barra de estado de la UI.
        movement_finished(): El movimiento terminó (convergió o agotó
            iteraciones); no se emite si el movimiento abortó.
        input_enabled(): La UI puede habilitar la entrada.
        worker_ready(): El worker abrió serial y arrancó telemetría.
        port_released(): El puerto serial quedó liberado.
    """

    pid_iteration = pyqtSignal(float, list, list)
    status_changed = pyqtSignal(str)
    movement_finished = pyqtSignal()
    input_enabled = pyqtSignal()
    worker_ready = pyqtSignal()
    port_released = pyqtSignal()

    def __init__(self, owner_id, telemetry_init=None):
        super().__init__()
        self._owner_id = owner_id
        self._links = list(LINKS_DEF)
        self._com_port = None

        # Estado de ejecución (port de los workers originales)
        self._running = False
        self._telemetry_running = False
        self._pid_abort = False
        self._gate = LoopGate()  # Sustituye a _pause_event/_t_resume_pending

        # Telemetría del robot (espacio servo 0-300)
        self._telemetry_lock = threading.Lock()
        inicio = telemetry_init if telemetry_init is not None else [150.0] * 6
        self._current_pos = list(inicio)
        self._last_valid = list(inicio)
        self._jump_freeze_count = [0] * 6

        # Feedback del gemelo (grados canónicos) desde PyBullet
        self._sim_lock = threading.Lock()
        self._sim_positions_deg = None
        bus.sim_model_position_signal.connect(self._on_sim_model_positions)

        # Gráficas
        self._last_pos_for_plot = None
        self._last_target_for_plot = None
        self._last_emit_time = 0.0
        self._session_start_time = time.time()

        # Cola de trabajos
        self._work_queue = queue.Queue()

        # Ganancias PID y garra
        self._kp = np.array([0.6, 0.45, 0.495])
        self._ki = np.array([1.4522, 0.1, 0.9746])
        self._kd = np.array([0.0619, 0.01, 0.0376])
        self._claw_mm = 30
        self._last_sent_claw_angle = -999
        self._last_commanded_angles = None
        self._claw_lock = threading.Lock()
        self.gripper_control_enabled = True

        # Casillas PID por planta (apagadas por defecto: comando directo)
        self._pid_robot = False
        self._pid_sim = False

    # ------------------------------------------------------------------ #
    #                     CASILLAS Y GANANCIAS PID                        #
    # ------------------------------------------------------------------ #

    def set_pid_targets(self, robot=None, sim=None):
        """Define qué plantas corren lazo PID cerrado.

        Args:
            robot (bool): None no modifica; True/False fija el lazo del
                robot (feedback: telemetría del MCU).
            sim (bool): None no modifica; True/False fija el lazo del
                gemelo (feedback: pose de PyBullet).
        """
        if robot is not None:
            self._pid_robot = bool(robot)
        if sim is not None:
            self._pid_sim = bool(sim)

    def get_pid_targets(self):
        """Retorna las casillas PID activas.

        Returns:
            dict: {"robot": bool, "sim": bool}.
        """
        return {"robot": self._pid_robot, "sim": self._pid_sim}

    def set_pid_gains(self, kp, ki, kd):
        """Define las ganancias del lazo."""
        self._kp = np.array(kp, dtype=np.float64)
        self._ki = np.array(ki, dtype=np.float64)
        self._kd = np.array(kd, dtype=np.float64)

    def set_claw_value(self, value):
        """Define la apertura de garra en mm."""
        with self._claw_lock:
            self._claw_mm = value

    def enable_gripper_control(self):
        self.gripper_control_enabled = True

    def disable_gripper_control(self):
        self.gripper_control_enabled = False

    # ------------------------------------------------------------------ #
    #                      PAUSA / ABORTO / RESET                         #
    # ------------------------------------------------------------------ #

    def pause(self):
        """Pausa el lazo de control."""
        self._gate.pause()

    def resume(self):
        """Reanuda el lazo y reinicia el dt del siguiente paso."""
        self._gate.resume()

    def reset_state(self):
        """Aborta el lazo activo, vacía la cola y reanuda la puerta."""
        self._pid_abort = True
        with self._work_queue.mutex:
            self._work_queue.queue.clear()
        self._gate.resume()
        self._session_start_time = time.time()
        self._last_emit_time = 0.0

    def abort_pid(self):
        self.reset_state()

    def force_abort(self):
        """Forzar detención inmediata e interrupción de IO serial."""
        self._running = False
        self._telemetry_running = False
        self._pid_abort = True
        self._gate.resume()
        self._interrumpir_io()
        with self._work_queue.mutex:
            self._work_queue.queue.clear()
        self.quit()
        self.wait(1000)

    def release_and_cleanup(self):
        """Libera el puerto serial, limpia buffers y notifica."""
        self._liberar_serial()
        self.port_released.emit()

    def _interrumpir_io(self):
        """Interrumpe las operaciones serial bloqueantes."""
        ser = serial_ports.get_serial()
        if ser:
            try:
                ser.cancel_read()
                ser.cancel_write()
                ser.close()
            except Exception:
                pass

    def _liberar_serial(self):
        """Liberación por defecto: interrumpir IO sin soltar el acceso."""
        self._interrumpir_io()

    def set_com_port(self, com_port):
        self._com_port = com_port

    def open_serial_manual(self, com_port):
        """Abre el puerto serial sin iniciar el bucle de ejecución."""
        self._com_port = com_port
        return self._open_serial(com_port)

    # ------------------------------------------------------------------ #
    #                 BUFFER DE FEEDBACK DEL GEMELO (SIM)                 #
    # ------------------------------------------------------------------ #

    @pyqtSlot(list, dict)
    def _on_sim_model_positions(self, motor_positions, _sphere_positions):
        """Guarda la pose de PyBullet convertida a grados canónicos.

        La señal llega en radianes del espacio PyBullet; los índices 4 y 5
        se niegan para volver al espacio canónico (inverso de la negación
        que ``set_joint_positions`` aplica al escribir).

        Args:
            motor_positions (list): Posiciones articulares en radianes.
            _sphere_positions (dict): Posiciones de esferas (sin uso).
        """
        deg = rad_to_deg(motor_positions)
        canonicos = [deg[0], deg[1], deg[2], deg[3], -deg[4], -deg[5]]
        with self._sim_lock:
            self._sim_positions_deg = canonicos

    def _feedback_sim_deg(self):
        """Pose del gemelo en grados canónicos; None si aún no hay dato."""
        with self._sim_lock:
            if self._sim_positions_deg is None:
                return None
            return list(self._sim_positions_deg)

    def _feedback_robot_deg(self):
        """Pose del robot en grados canónicos; None sin telemetría."""
        raw = self._read_positions()
        if any(v is None for v in raw):
            return None
        return list(robotang_angulos(*raw))

    def _esperar_feedback_sim(self, timeout=2.0):
        """Espera breve a la primera pose del gemelo.

        Args:
            timeout (float): Máximo de espera en segundos.

        Returns:
            bool: True si llegó feedback del gemelo.
        """
        limite = time.time() + timeout
        while time.time() < limite:
            if not self._running or self._pid_abort:
                return False
            if self._feedback_sim_deg() is not None:
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------ #
    #                    DISPONIBILIDAD DE PLANTAS                        #
    # ------------------------------------------------------------------ #

    def _serial_disponible(self):
        """True si este worker es dueño de un puerto serial abierto."""
        ser = serial_ports.get_serial()
        return bool(serial_ports.is_active(self._owner_id)
                    and ser is not None and ser.is_open)

    def _sim_disponible(self):
        """True si el gemelo (PyBullet) está corriendo."""
        return bus.sim_is_running()

    # ------------------------------------------------------------------ #
    #                          SINKS DE ENVÍO                             #
    # ------------------------------------------------------------------ #

    def _enviar_lazo_robot(self, q_deg6):
        """Envía un comando canónico al robot (vía serial)."""
        self._enviar_robot(angulos_robotang(*q_deg6))

    def _enviar_lazo_sim(self, q_deg6):
        """Envía un comando canónico al gemelo (PyBullet en radianes)."""
        bus.sim_update_pybullet_signal.emit(deg_to_rad(q_deg6).tolist())

    def _despachar_directo(self, q_deg6):
        """Envía un comando directo (sin lazo) a todas las plantas vivas.

        Args:
            q_deg6 (list): Comando en grados canónicos.
        """
        if self._serial_disponible():
            self._enviar_lazo_robot(q_deg6)
        if self._sim_disponible():
            self._enviar_lazo_sim(q_deg6)
        self._last_commanded_angles = list(q_deg6)

    # ------------------------------------------------------------------ #
    #                 ORQUESTADOR DE MOVIMIENTO MULTI-PLANTA              #
    # ------------------------------------------------------------------ #

    def _run_movement(self, target_xyz, limites_deg, angulo_garra,
                      lazo_robot=True, lazo_sim=True, max_iter=800,
                      tolerancias=None, t_start=None):
        """Ejecuta un movimiento cartesiano sobre las plantas disponibles.

        Construye las plantas según conectividad (serial propio / gemelo
        corriendo) y las casillas PID; cada planta con lazo corre su propio
        PID contra su propio feedback y las demás reciben una solución de
        IK directa. El fin lo decide la planta de avance (robot si hay
        serial, si no gemelo).

        Args:
            target_xyz (list): Objetivo cartesiano [x, y, z] en mm.
            limites_deg (list): Límites físicos por articulación.
            angulo_garra (float): Ángulo de garra del comando.
            lazo_robot (bool): Casilla "Aplicar PID al robot".
            lazo_sim (bool): Casilla "Aplicar PID al gemelo virtual".
            max_iter (int): Máximo de iteraciones del movimiento.
            tolerancias (list): Tolerancias [x, y, z] en mm.
            t_start (float): Marca de tiempo inicial compartida.

        Returns:
            bool: True si la planta de avance convergió.
        """
        plantas = {}
        if self._serial_disponible():
            plantas['robot'] = PlantPID(
                self._feedback_robot_deg, self._enviar_lazo_robot,
                lazo_robot)
        if self._sim_disponible():
            if (self._feedback_sim_deg() is not None
                    or self._esperar_feedback_sim()):
                plantas['sim'] = PlantPID(
                    self._feedback_sim_deg, self._enviar_lazo_sim, lazo_sim)

        if not plantas:
            # Sin plantas alcanzables: nada que mover.
            self.movement_finished.emit()
            return False

        planta_avance = 'robot' if 'robot' in plantas else 'sim'
        ok = run_movement(
            target_xyz, limites_deg, angulo_garra, plantas, planta_avance,
            gains={"kp": self._kp, "ki": self._ki, "kd": self._kd},
            links=self._links, max_iter=max_iter, tolerancias=tolerancias,
            gate=self._gate,
            should_abort=lambda: not self._running or self._pid_abort,
            on_iteration=self.actualizar_grafica, t_start=t_start)

        # Port de la semántica original: converge y max-iteraciones emiten;
        # un movimiento abortado no emite.
        if self._running and not self._pid_abort:
            self.movement_finished.emit()
        return ok

    # ------------------------------------------------------------------ #
    #                COMUNICACION SERIAL PROPIA (0-300)                   #
    # ------------------------------------------------------------------ #

    def _open_serial(self, com_port):
        return serial_ports.request_access(com_port, self._owner_id)

    def _close_serial(self):
        serial_ports.release_access(self._owner_id)

    def _enviar_robot(self, q_servos):
        """Envía la trama A-F de servos (0-300) por el puerto propio."""
        com = self._com_port or serial_ports._com or "COM7"
        if not serial_ports.is_active(self._owner_id):
            try:
                serial_ports.request_access(com, self._owner_id)
            except Exception:
                # print(f"[DEBUG] [{self._owner_id}] Error solicitando acceso "
                #       f"a {com}: {e}")
                return

        ser = serial_ports.get_serial()
        if ser is None or not ser.is_open:
            try:
                serial_ports.request_access(com, self._owner_id)
            except Exception as e:
                print(
                    f"[DEBUG] [{self._owner_id}] Error reabriendo {com}: {e}")

        if not serial_ports.get_serial() or not serial_ports.get_serial().is_open:
            print("[DEBUG] Serial port not open, cannot send command.")
            return

        try:
            trama = ""
            motores = ['A', 'B', 'C', 'D', 'E', 'F']
            for i, char in enumerate(motores):
                val_pwm = int(round(
                    max(0, min(300, float(q_servos[i]))) * (1023 / 300)))
                trama += f"{char}{val_pwm}"
            trama += "\n"
            serial_ports.safe_write(trama.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            print(f"Error enviando comando: {e}")

    def _telemetry_reader(self):
        """Hilo lector de telemetría (espacio servo 0-300).

        Port del lector original: valida rango, descarta tramas nulas
        (caídas de tensión) y congela saltos > 35 unidades durante las
        primeras 4 apariciones para filtrar ruido electromagnético.
        """
        pattern = re.compile(r"([A-F])(\d+\.?\d*)T[A-F](\d+)")
        congelado_count = self._jump_freeze_count

        while self._telemetry_running:
            try:
                if not serial_ports.is_active(self._owner_id):
                    time.sleep(0.1)
                    continue

                ser = serial_ports.get_serial()
                if ser is None or not ser.is_open:
                    time.sleep(0.1)
                    continue

                if serial_ports.get_in_waiting() > 0:
                    raw_line = serial_ports.safe_readline()
                    line = raw_line.decode('ascii', errors='ignore').strip()
                    if not line:
                        continue

                    matches = pattern.findall(line)
                    if len(matches) < 6:
                        continue

                    temp_pos = [None] * 6
                    for motor_char, pos_val, _ in matches:
                        idx = ord(motor_char) - ord('A')
                        if idx < 6:
                            temp_pos[idx] = float(pos_val)

                    with self._telemetry_lock:
                        # Validación de rango físico individual
                        for i in range(6):
                            if temp_pos[i] is not None and not (
                                    0 <= temp_pos[i] <= 300):
                                temp_pos[i] = (self._last_valid[i]
                                               if self._last_valid[i] is not None
                                               else 150.0)

                        # Detección de tramas nulas / caídas de tensión
                        if all(v is not None and abs(v) < 0.001
                               for v in temp_pos[:4]):
                            continue

                        # Filtro anti-ruido con corrección individual
                        for i in range(6):
                            if temp_pos[i] is not None:
                                if self._last_valid[i] is not None:
                                    diff = abs(temp_pos[i]
                                               - self._last_valid[i])
                                    if diff > 35.0:
                                        congelado_count[i] += 1
                                        if congelado_count[i] <= 4:
                                            temp_pos[i] = self._last_valid[i]
                                        else:
                                            congelado_count[i] = 0
                                    else:
                                        congelado_count[i] = 0

                        # Actualización limpia de la telemetría
                        for i in range(6):
                            if temp_pos[i] is not None:
                                self._current_pos[i] = temp_pos[i]
                                self._last_valid[i] = temp_pos[i]

            except Exception as e:
                print(f"Alerta: Hilo de telemetria interrumpido: {e}")
                break

    def _read_positions(self):
        """Lee la telemetría de forma segura (espacio servo 0-300)."""
        with self._telemetry_lock:
            return list(self._current_pos)

    def _wait_for_position(self, target, timeout=5.0):
        """Espera (espacio servo 0-300) a que la telemetría llegue al objetivo.

        Args:
            target (list): Posiciones servo objetivo.
            timeout (float): Máximo de espera en segundos.

        Returns:
            bool: True si la telemetría llegó a menos de 5 unidades.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._running or self._pid_abort:
                return False
            self._gate.wait()
            current_pos = self._read_positions()
            if all(current_pos[i] is not None
                   and abs(current_pos[i] - target[i]) < 5
                   for i in range(len(target))):
                return True
            time.sleep(0.1)
        print("Timeout esperando posición:", target)
        return False

    def _wait_sim_reach(self, q_deg6, timeout=6.0, tolerancia=2.0):
        """Espera a que el gemelo alcance una pose articular canónica.

        Args:
            q_deg6 (list): Pose objetivo en grados canónicos.
            timeout (float): Máximo de espera en segundos.
            tolerancia (float): Tolerancia articular en grados.

        Returns:
            bool: True si el gemelo llegó a la pose.
        """
        limite = time.time() + timeout
        indices = (0, 1, 2, 4)
        while time.time() < limite:
            if not self._running or self._pid_abort:
                return False
            self._gate.wait()
            fb = self._feedback_sim_deg()
            if fb is not None and all(
                    abs(fb[i] - q_deg6[i]) < tolerancia for i in indices):
                return True
            time.sleep(0.05)
        return False

    def _arrancar_telemetria(self):
        """Abre serial si falta y arranca el hilo de telemetría.

        Returns:
            bool: True si el serial quedó activo (o ya lo estaba); False
                en modo virtual (sin COM) o si la apertura falló.
        """
        ser = serial_ports.get_serial()
        if ser is None or not ser.is_open:
            if not self._com_port:
                return False
            if not self._open_serial(self._com_port):
                return False
        self._telemetry_running = True
        threading.Thread(target=self._telemetry_reader, daemon=True).start()
        self.worker_ready.emit()
        time.sleep(1)
        return True

    # ------------------------------------------------------------------ #
    #                           GRÁFICAS                                  #
    # ------------------------------------------------------------------ #

    def actualizar_grafica(self, target_xyz, pos_actual_xyz):
        """Emite pid_iteration para la gráfica cartesiana.

        Args:
            target_xyz (list): Objetivo cartesiano [x, y, z] en mm.
            pos_actual_xyz (list): Posición actual [x, y, z] en mm.
        """
        self._last_target_for_plot = list(target_xyz)
        self._last_pos_for_plot = list(pos_actual_xyz)
        t_relativo = round(time.time() - self._session_start_time, 4)
        self.pid_iteration.emit(
            t_relativo, self._last_pos_for_plot, self._last_target_for_plot)

    def _update_graph_with_angles(self, target_angles_deg):
        """Calcula la FK del objetivo y de la telemetría y actualiza la gráfica.

        Args:
            target_angles_deg (list): Ángulos objetivo en grados canónicos.
        """
        if (target_angles_deg == [0, 0, 0, 0, 0, 0]) or (
                target_angles_deg is None):
            target_xyz = [0, 0, 468]
            pos_actual_xyz = [0, 0, 468]
            self.actualizar_grafica(target_xyz, pos_actual_xyz)
            return

        target_rad = np.radians([target_angles_deg[0], target_angles_deg[1],
                                 target_angles_deg[2], target_angles_deg[4]])
        target_xyz = cinematica_directa(target_rad, self._links)

        # Posición actual: telemetría del robot; sin serial se usa el
        # propio comando (port de los workers, con guardia anti-None).
        fb = self._feedback_robot_deg()
        if fb is None:
            fb = list(target_angles_deg)
        pos_rad = np.radians([fb[0], fb[1], fb[2], fb[4]])
        pos_actual_xyz = cinematica_directa(pos_rad, self._links)

        self.actualizar_grafica(target_xyz.tolist(), pos_actual_xyz.tolist())
