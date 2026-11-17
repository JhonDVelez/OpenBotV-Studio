"""
Módulo que orquesta el PickAndPlaceWorker como un QThread independiente.

El worker reutiliza la base compartida ``ControlWorkerBase``
(src/services/robot/control_worker_base.py) —serial propio, telemetría,
feedback del gemelo y orquestador multi-planta— y añade la máquina de
estados de la secuencia de pick and place. Funciona con robot físico
(serial propio) o en modo virtual (solo gemelo, sin serial).
"""

import math
import queue
import time

import numpy as np
from PyQt6.QtCore import QTimer, pyqtSignal, pyqtSlot

from src.features.kinematics.coordinate_correction import apertura_de_garra
from src.features.pick_and_place.pick_place_states import PickPlaceState
from src.features.pick_and_place.pick_place_state_machine import PickPlaceStateMachine
from src.features.pick_and_place.logic.context import PickPlaceContext
from src.services.robot.control_worker_base import (
    ControlWorkerBase,
    LIMITES_HOME,
    LIMITES_TARGET,
)
from src.services.robot.serial_manager import serial_ports


class PickAndPlaceWorker(ControlWorkerBase):
    """
    Worker independiente para Pick and Place con máquina de estados,
    conexión serial propia (con robot) y bucles PID sobre el motor
    compartido. En modo virtual ejecuta la secuencia solo contra el
    gemelo de PyBullet.
    """

    sequence_completed = pyqtSignal()
    sequence_failed = pyqtSignal(str)
    status_message_updated = pyqtSignal(str)
    mode_change_requested = pyqtSignal(str)
    vision_search_request = pyqtSignal(bool, bool)

    def __init__(self, kinematics_controller=None):
        super().__init__("PickAndPlaceWorker", telemetry_init=[None] * 6)
        self.context = PickPlaceContext()
        self.kinematics_controller = kinematics_controller
        self._sm = PickPlaceStateMachine(on_state_change=self._on_state_change)

        self.skip_home = False

        self._check_timer = QTimer()
        self._check_timer.setSingleShot(False)
        self.TIEMPO_PASO_GARRA = 1000
        self._pending_callback = None
        self._settle_estado = {'limite': 0.0, 'ultima': None}

        self.movement_finished.connect(self._on_movement_finished)

    # ------------------------------------------------------------------ #
    #                      PAUSA / RESET / LIMPIEZA                       #
    # ------------------------------------------------------------------ #

    def pause(self):
        """Pausa el lazo y el temporizador de asentamiento."""
        super().pause()
        self._check_timer.stop()

    def resume(self):
        """Reanuda el lazo y re-programa la espera pendiente."""
        super().resume()
        if self._pending_callback:
            self._wait_for_settle(self._pending_callback)

    def stop_and_reset(self):
        """Detiene la secuencia y limpia el estado para un reseteo limpio."""
        if self.current_state_value != 'idle':
            self._sm.reset()
        self._check_timer.stop()
        self.context.reset()
        self.reset_state()

    def _liberar_serial(self):
        """Variante P&P: soltar el acceso al puerto (además de cerrarlo)."""
        serial_ports.release_access(self._owner_id)

    # ------------------------------------------------------------------ #
    #                    ARRANQUE / COLA DE TRABAJOS                      #
    # ------------------------------------------------------------------ #

    def start_worker(self, com_port):
        """Arranca el hilo con conexión serial propia."""
        self._com_port = com_port
        self._running = True
        self._pid_abort = False
        self.start()

    def start_virtual(self, com_port=None):
        """Arranca el hilo en modo virtual (sin robot, solo gemelo)."""
        self._com_port = com_port
        self._running = True
        self._pid_abort = False
        self.start()

    def execute_pid_only(self, target_xyz, limites_deg, max_iter=800,
                         tolerancias=None, t_start=None, angulo_garra=None):
        self._work_queue.put(('execute_pid_only', target_xyz, limites_deg,
                              max_iter, tolerancias, t_start, angulo_garra))

    def execute_direct_move(self, positions):
        """Encola un movimiento directo (grados canónicos) a las plantas."""
        self._work_queue.put(('direct_move', positions))

    def run(self):
        con_serial = self._arrancar_telemetria()
        if not con_serial:
            if self._com_port:
                self.movement_finished.emit()
                return
            # Modo virtual (sin robot): la secuencia opera sobre el gemelo
            self.worker_ready.emit()

        while self._running:
            try:
                work = self._work_queue.get(timeout=0.01)
            except queue.Empty:
                continue

            if work[0] == 'execute_pid_only':
                _, target, limites, max_iter, tol, t_start, angulo_garra = work
                if angulo_garra is None:
                    with self._claw_lock:
                        angulo_garra = apertura_de_garra(self._claw_mm)
                self._pid_abort = False
                self._run_movement(
                    target, limites, angulo_garra, self._pid_robot,
                    self._pid_sim, max_iter=max_iter, tolerancias=tol,
                    t_start=t_start)
            elif work[0] == 'direct_move':
                _, positions = work
                self._despachar_directo(positions)
                self._update_graph_with_angles(positions)
                self.movement_finished.emit()
            elif work[0] == 'stop':
                break

        self._telemetry_running = False
        self._close_serial()

    # ------------------------------------------------------------------ #
    #                        MÁQUINA DE ESTADOS                           #
    # ------------------------------------------------------------------ #

    @pyqtSlot(dict)
    def on_poses_from_camera(self, poses):
        self.context.sphere_poses.update(poses)

    def _on_state_change(self, state_name):

        if state_name == PickPlaceState.WAITING_FOR_INPUT.value:
            if self.context.ik_target is None:
                self.vision_search_request.emit(True, True)
            else:
                self.vision_search_request.emit(True, False)
        else:
            self.vision_search_request.emit(False, False)

        handlers = {
            PickPlaceState.HOME1_MOVE.value: self._enter_home1_move,
            PickPlaceState.HOME1_VALIDATE.value: self._enter_home1_validate,
            PickPlaceState.PID_HOME.value: self._enter_pid_home,
            PickPlaceState.PICK_APPROACH.value: self._enter_pick_approach,
            PickPlaceState.PICK_DOWN.value: self._enter_pick_down,
            PickPlaceState.PICK_GRASP.value: self._enter_pick_grasp,
            PickPlaceState.RETRACT_LIFT.value: self._enter_retract_lift,
            PickPlaceState.RETRACT_TO_HOME1_PICK.value: self._enter_retract_to_home1_pick,
            PickPlaceState.PID_HOME_PLACE.value: self._enter_pid_home_place,
            PickPlaceState.PLACE_APPROACH.value: self._enter_place_approach,
            PickPlaceState.PLACE_DOWN.value: self._enter_place_down,
            PickPlaceState.PLACE_RELEASE.value: self._enter_place_release,
            PickPlaceState.RETRACT_FROM_PLACE.value: self._enter_retract_from_place,
            PickPlaceState.FINAL_SEQ_HOME.value: self._enter_final_seq_home,
            PickPlaceState.FINAL_SEQ_HOME2.value: self._enter_final_seq_home2,
            PickPlaceState.FINAL_SEQ_HOME1.value: self._enter_final_seq_home1,
            PickPlaceState.WAITING_FOR_INPUT.value: self._enter_waiting_for_input,
        }
        handler = handlers.get(state_name)
        if handler:
            handler()

    def calcular_angulo_garra(self, tam):
        return apertura_de_garra(tam)

    def _wait_for_settle(self, callback):
        """Espera el asentamiento de la planta antes del siguiente paso.

        Con robot: temporizador fijo de 2500 ms. Sin robot: sondeo no
        bloqueante de la pose del gemelo (asentada cuando deja de variar
        o vence el plazo de 6 s).

        Args:
            callback: Callback a invocar cuando la planta se asentó.
        """
        self._pending_callback = callback

        def wrapped_callback():
            self._pending_callback = None
            callback()

        try:
            self._check_timer.timeout.disconnect()
        except Exception:
            pass

        if self._serial_disponible():
            self._check_timer.setSingleShot(True)
            self._check_timer.timeout.connect(wrapped_callback)
            self._check_timer.start(2500)
            return

        # Sondeo del asentamiento del gemelo (hilo de UI, no bloqueante)
        self._settle_estado = {'limite': time.time() + 6.0, 'ultima': None}

        def paso_sondeo():
            fb = self._feedback_sim_deg()
            estado = self._settle_estado
            asentado = (fb is not None and estado['ultima'] is not None
                        and np.allclose(fb, estado['ultima'], atol=0.05))
            if not asentado:
                estado['ultima'] = fb
                asentado = time.time() >= estado['limite']
            if asentado:
                self._check_timer.stop()
                try:
                    self._check_timer.timeout.disconnect(paso_sondeo)
                except Exception:
                    pass
                wrapped_callback()

        self._check_timer.setSingleShot(False)
        self._check_timer.timeout.connect(paso_sondeo)
        self._check_timer.start(100)

    def _enviar_comando_movimiento(self, apertura):
        """Mueve la garra manteniendo el resto de articulaciones."""
        if self._last_commanded_angles is not None:
            current_angles = list(self._last_commanded_angles)
        else:
            fb = self._feedback_robot_deg()
            if fb is None:
                return
            current_angles = list(fb)
        ang_garra = self.calcular_angulo_garra(apertura)
        current_angles[5] = ang_garra
        self._last_commanded_angles = current_angles
        self.execute_direct_move(current_angles)

    def _ejecutar_paso_apertura(self, apertura_actual, apertura_final, incremento):
        nueva_apertura = min(apertura_actual + incremento, apertura_final)
        self._enviar_comando_movimiento(nueva_apertura)
        if nueva_apertura < apertura_final:
            QTimer.singleShot(self.TIEMPO_PASO_GARRA, lambda: self._ejecutar_paso_apertura(
                nueva_apertura, apertura_final, incremento))
        else:
            self._sm.place_release_done()

    def _enter_home1_move(self):
        servos_paso1 = [0, 0, 130, 0, -30, 0]
        self.execute_direct_move(servos_paso1)

        def ejecutar_paso_2():
            servos_paso2 = [0, -90, 130, 0, -30, 0]
            self.execute_direct_move(servos_paso2)

            def finalizar_home1():
                self._sm.home1_done()

            self._wait_for_settle(finalizar_home1)

        self._wait_for_settle(ejecutar_paso_2)

    def _enter_home1_validate(self):
        self._wait_for_settle(self._sm.home1_validated)

    def _enter_waiting_for_input(self):
        if self.context.ik_target is None:
            self.status_message_updated.emit("Selecciona objeto y su color")
            self.mode_change_requested.emit('pick')
        else:
            self.status_message_updated.emit("Selecciona destino en el plano")
            self.mode_change_requested.emit('place')

    def _enter_pid_home(self, angulo_garra_custom=None):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = angulo_garra_custom if angulo_garra_custom is not None else self.calcular_angulo_garra(
            tam + 50)
        self.update_gains_from_panel()
        tx, ty, tz = 185, 0, 170
        tz = self.corregir_z(tx, ty, tz)
        tx, ty = self.corregir_xy(tx, ty)
        self.execute_pid_only([tx, ty, tz], LIMITES_HOME,
                              angulo_garra=ang_garra)

    def _enter_pick_approach(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam + 50)
        self.update_gains_from_panel()
        x, y, z = self.context.ik_target
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = tam
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        # Aproximar primero en Y y Z (retrasado en X)
        self.execute_pid_only(
            [x-60, y, z], LIMITES_TARGET, angulo_garra=ang_garra)

    def _enter_pick_down(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam + 50)
        self.update_gains_from_panel()
        x, y, z = self.context.ik_target
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = tam + 5
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        # print(f"[DEBUG] Pick Down Target: x={x}, y={y}, z={z}, ang_garra={ang_garra}")
        # Luego aproximar en X a la posición final
        self.execute_pid_only([x, y, z], LIMITES_TARGET,
                              angulo_garra=ang_garra)

    def _enter_pick_grasp(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        apertura = tam - 20
        self._enviar_comando_movimiento(apertura)
        self._wait_for_settle(self._sm.pick_grasp_done)

    def _enter_retract_lift(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam - 20)
        self.update_gains_from_panel()
        x, y, z = self.context.ik_target
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = 100
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        self.execute_pid_only([x, y, z], LIMITES_TARGET,
                              angulo_garra=ang_garra)

    def _enter_retract_to_home1_pick(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam - 20)
        servos = [0, -90, 130, 0, -30, ang_garra]
        self.execute_direct_move(servos)
        self._wait_for_settle(self._sm.retract_to_home1_pick_done)

    def _enter_pid_home_place(self, angulo_garra_custom=None):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = angulo_garra_custom if angulo_garra_custom is not None else self.calcular_angulo_garra(
            tam - 20)
        self.update_gains_from_panel()
        tx, ty, tz = 185, 0, 170
        tz = self.corregir_z(tx, ty, tz)
        tx, ty = self.corregir_xy(tx, ty)
        self.execute_pid_only([tx, ty, tz], LIMITES_HOME,
                              angulo_garra=ang_garra)

    def _enter_place_approach(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam - 20)
        self.update_gains_from_panel()
        if self.context.place_target_coords is None:
            print("[ERROR] place_target_coords is None in _enter_place_approach")
            self.sequence_failed.emit(
                'No se ha seleccionado posición de colocación (place)')
            self._sm.reset()
            return
        x, y, z = self.context.place_target_coords
        x = float(self.context.place_target_coords['x'])
        y = float(self.context.place_target_coords['y'])
        z = float(self.context.place_target_coords['z'])
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        desz = tam + 10 if np.sqrt(x**2 + (y+110)**2) < 190 else tam*2
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = desz
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        self.execute_pid_only(
            [x-60, y, z], LIMITES_TARGET, angulo_garra=ang_garra)

    def _enter_place_down(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam - 20)
        self.update_gains_from_panel()
        if self.context.place_target_coords is None:
            print("[ERROR] place_target_coords is None in _enter_place_down")
            self.sequence_failed.emit(
                'No se ha seleccionado posición de colocación (place)')
            self._sm.reset()
            return
        x, y, z = self.context.place_target_coords
        x = float(self.context.place_target_coords['x'])
        y = float(self.context.place_target_coords['y'])
        z = float(self.context.place_target_coords['z'])
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        # desz = tam + 10 if np.sqrt(x**2 + (y+110)**2) < 190 else tam*2
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = tam*2
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        if y < 0:
            y = y - 20
            x = x
        else:
            y = y
            x = x + 20
        self.execute_pid_only([x, y, z], LIMITES_TARGET,
                              angulo_garra=ang_garra)

    def _enter_place_release(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        apertura_total = tam + 20
        self._enviar_comando_movimiento(apertura_total)
        self._wait_for_settle(self._sm.place_release_done)

    def _enter_retract_from_place(self):
        tam = self.context.tamano_seleccionado if self.context.tamano_seleccionado else 30
        ang_garra = self.calcular_angulo_garra(tam + 20)
        self.update_gains_from_panel()
        if self.context.place_target_coords is None:
            print("[ERROR] place_target_coords is None in _enter_place_down")
            self.sequence_failed.emit(
                'No se ha seleccionado posición de colocación (place)')
            self._sm.reset()
            return
        x, y, z = self.context.place_target_coords
        x = float(self.context.place_target_coords['x'])
        y = float(self.context.place_target_coords['y'])
        z = float(self.context.place_target_coords['z'])
        des = 1 if np.sqrt(x**2 + (y+110)**2) < 190 else 3.0
        r = tam + des
        angle = math.atan((x)/(y+110))
        x = x + r*math.sin(1.5708 - angle)
        y = y - r*math.cos(1.5708 - angle)
        x1 = y + 110
        y1 = x
        z = 100
        z = self.corregir_z(x1, y1, z)
        x, y = self.corregir_xy(x1, y1)
        self.execute_pid_only([x, y, z], LIMITES_TARGET,
                              angulo_garra=ang_garra)

    def _enter_final_seq_home1(self):
        servos = [0, -90, 130, 0, -30, 0]
        self.execute_direct_move(servos)
        self._wait_for_settle(self._final_home1_cleanup)

    def _final_home1_cleanup(self):
        self.context.reset()
        self._sm.final_home1_done()
        self.sequence_completed.emit()

    def _enter_final_seq_home(self):
        self._sm.final_home_done()

    def _enter_final_seq_home2(self):
        self._sm.final_home2_done()

    def corregir_xy(self, x, y):
        from src.features.kinematics.coordinate_correction import corregir_xy as corr_xy
        return corr_xy(x, y)

    def corregir_z(self, x, y, z):
        from src.features.kinematics.coordinate_correction import corregir_z as corr_z
        return corr_z(x, y, z)

    # ------------------------------------------------------------------ #
    #                     ENTRADA DEL USUARIO (UI)                        #
    # ------------------------------------------------------------------ #

    @pyqtSlot(str)
    def pick(self, color):
        self.update_gains_from_panel()
        self.context.selected_color = color

        if color in self.context.sphere_poses:
            pose_data = self.context.sphere_poses[color]
            if isinstance(pose_data, dict) and 'position' in pose_data:
                self.context.ik_target = pose_data['position']
            else:
                self.context.ik_target = pose_data

        if self._sm.current_state_value == PickPlaceState.WAITING_FOR_INPUT.value:
            self._sm.ready_to_home2()
        elif self._sm.current_state_value == PickPlaceState.IDLE.value:
            self._sm.start_op()

    @pyqtSlot(dict)
    def place(self, coords):
        self.update_gains_from_panel()
        self.context.place_target_coords = coords

        if self._sm.current_state_value == PickPlaceState.WAITING_FOR_INPUT.value:
            if self.context.ik_target is not None:
                self._sm.ready_to_place()
            else:
                self._sm.ready_to_home2()
        elif self._sm.current_state_value == PickPlaceState.IDLE.value:
            self._sm.start_op()

    @pyqtSlot()
    def _on_movement_finished(self):
        current = self.current_state_value
        if current == PickPlaceState.PID_HOME.value:
            self._sm.pid_home_done()
        elif current == PickPlaceState.PICK_APPROACH.value:
            self._sm.pick_approach_done()
        elif current == PickPlaceState.PICK_DOWN.value:
            self._sm.pick_down_done()
        elif current == PickPlaceState.RETRACT_LIFT.value:
            self._sm.retract_lift_done()
        elif current == PickPlaceState.PID_HOME_PLACE.value:
            self._sm.pid_home_place_done()
        elif current == PickPlaceState.PLACE_APPROACH.value:
            self._sm.place_approach_done()
        elif current == PickPlaceState.PLACE_DOWN.value:
            self._sm.place_down_done()
        elif current == PickPlaceState.RETRACT_FROM_PLACE.value:
            self._sm.retract_from_place_done()
        elif current == PickPlaceState.FINAL_SEQ_HOME.value:
            self._sm.final_home_done()
        elif current == PickPlaceState.FINAL_SEQ_HOME2.value:
            self._sm.final_home2_done()

    @pyqtSlot()
    def abort(self):
        if self.current_state_value != PickPlaceState.IDLE.value:
            self._sm.reset()

    def update_pid_gains(self, gains):
        self.set_pid_gains(gains['kp'], gains['ki'], gains['kd'])

    def update_gains_from_panel(self):
        """Toma ganancias y casillas PID del panel de cinemática."""
        if self.kinematics_controller:
            try:
                widget = self.kinematics_controller.get_widget()
                gains = widget.get_pid_gains()
                self.set_pid_gains(gains["kp"], gains["ki"], gains["kd"])
                targets = widget.get_pid_targets()
                self.set_pid_targets(targets["robot"], targets["sim"])
            except Exception:
                pass

    def start_sequence(self):
        if self._sm.current_state_value == 'idle':
            self._sm.start_op()

    @property
    def current_state_value(self):
        return self._sm.current_state.value
