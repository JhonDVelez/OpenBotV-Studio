"""
Modulo que orquesta el flujo de control cinematico del robot.

Actua como puente entre la entrada de coordenadas cartesianas del usuario
(KinematicsWorker en hilo independiente) y el motor de control PID basado
en el codigo standalone Prueba_controlv11.

Flujo:
    1. Al activar modo cinematica:
       - Con robot: suspender serial del RobotWorker, HOME directo
         (sin PID) y esperar llegada (2.5s aprox).
       - Sin robot: iniciar el gemelo si estaba parado y enviar HOME
         virtual directo al gemelo (sin PID).
       - Sincronizar las casillas de plantas PID del panel.
       - Habilitar entrada de coordenadas del usuario.
    2. Al recibir coordenadas ("Enviar"):
       - Pasa al worker las casillas PID activas.
       - PID al HOME con compensaciones.
       - PID al TARGET con compensaciones.
    3. Al salir del modo cinematica:
       - Detener el worker.
       - Reanudar serial del RobotWorker (si se suspendió).

El modelo 3D (QML) sigue siempre la pose real del gemelo vía PyBullet;
este controlador ya no interpola ni emite posiciones articulares.
"""

from PyQt6.QtCore import QObject, pyqtSlot, QThread
from src.features.kinematics.kinematics_widget import KinematicsWidget
from src.features.kinematics.kinematics_worker import KinematicsWorker
from src.services.data.signals import bus
from src.services.data.enums import Modes
from src.services.ui.notification_manager import notification_manager
from src.services.data.enums.types import NotificationType
from .coordinate_correction import corregir_xy, corregir_z


class KinematicsController(QObject):
    """
    Controlador para el modo cinematico del brazo robotico.

    Orquesta el lifecycle completo:
    - Entrada al modo (HOME directo/virtual + habilitar UI)
    - Ejecucion de movimientos (PID HOME + PID TARGET, por planta)
    - Salida del modo (detener worker + reanudar RobotWorker)
    """

    def __init__(self, parent=None):
        super().__init__()
        self.kinematics_widget = KinematicsWidget(parent)
        self.kinematics_worker = None
        self._claw_mm = 30  # Valor inicial persistente
        self._robot_service = None
        self._graph_controller = None
        self._mode_active = False
        self._serial_suspendido = False

        self._create_worker()
        self.__setup_connections()

    def _create_worker(self):
        if hasattr(self, 'kinematics_worker') and self.kinematics_worker:
            try:
                self.kinematics_worker.status_changed.disconnect(
                    self._on_status_changed)
                self.kinematics_worker.movement_finished.disconnect(
                    self._on_movement_finished)
                self.kinematics_worker.input_enabled.disconnect(
                    self._on_input_enabled)
            except Exception:
                pass
            self.kinematics_worker.force_abort()

        self.kinematics_worker = KinematicsWorker()
        self.kinematics_worker.status_changed.connect(
            self._on_status_changed)
        self.kinematics_worker.movement_finished.connect(
            self._on_movement_finished)
        self.kinematics_worker.input_enabled.connect(
            self._on_input_enabled)

        if self._graph_controller:
            self._graph_controller.set_kinematics_worker(self.kinematics_worker)

    def set_robot_service(self, robot_service):
        self._robot_service = robot_service

    def set_graph_controller(self, graph_controller):
        self._graph_controller = graph_controller
        if self._graph_controller and self.kinematics_worker:
            self._graph_controller.set_kinematics_worker(self.kinematics_worker)

    def __setup_connections(self):
        self.kinematics_widget.send_clicked.connect(
            self.execute_kinematics)
        self.kinematics_widget.claw_changed.connect(
            self._on_claw_changed)

        bus.sld_change_mode_signal.connect(
            self._on_global_mode_changed)
        bus.sim_change_mode_signal.connect(
            self._on_global_mode_changed)
        bus.kin_change_mode_signal.connect(
            self._on_global_mode_changed)

    def _hay_robot(self):
        """True si hay robot físico conectado con servicio disponible."""
        return bus.phy_is_connected and self._robot_service is not None

    def _asegurar_simulacion(self):
        """Garantiza que el gemelo esté corriendo para el modo virtual.

        El arranque real lo gestiona el SimulationController vía el bus;
        se marca el estado de inmediato para que el worker pueda enviar
        comandos al gemelo sin esperar al siguiente ciclo del hilo de UI
        (PyBullet los aplica en cuanto arranca la simulación).
        """
        if not bus.sim_is_running():
            bus.sim_set_running(True)
            bus.sim_start_request.emit()

    def enter_kinematics_mode(self):
        if self._mode_active:
            return

        robot_conectado = self._hay_robot()
        self._mode_active = True
        # Instanciar worker fresco cada vez que se entra al modo
        self._create_worker()

        # Sincronizar valor persistente y casillas de plantas PID
        self.kinematics_widget.sync_pid_targets(robot_conectado)
        targets = self.kinematics_widget.get_pid_targets()
        self.kinematics_worker.set_claw_value(self._claw_mm)
        self.kinematics_worker.set_pid_targets(
            targets["robot"], targets["sim"])

        bus.kin_change_mode_signal.emit(Modes.KINEMATIC)

        self._set_inputs_enabled(False)

        if robot_conectado:
            self._serial_suspendido = True
            self._robot_service.suspend_serial()
            com = self._robot_service.get_com()
            self.kinematics_worker.send_home_direct(com)
        else:
            self._serial_suspendido = False
            self._asegurar_simulacion()
            notification_manager.notify(
                "Modo cinemática virtual: sin robot, se mueve solo el "
                "gemelo de PyBullet", NotificationType.TOAST_INFO)
            self.kinematics_worker.start_virtual()
            self.kinematics_worker.send_home_virtual()

    def exit_kinematics_mode(self):

        if not self._mode_active:
            return

        self._mode_active = False
        # Usar release_and_cleanup para cerrar y notificar
        if hasattr(self, 'kinematics_worker') and self.kinematics_worker:
            self.kinematics_worker.port_released.connect(
                self._on_kinematics_port_released)
            self.kinematics_worker.force_abort()
            self.kinematics_worker.release_and_cleanup()

    @pyqtSlot()
    def _on_kinematics_port_released(self):
        try:
            self.kinematics_worker.port_released.disconnect(
                self._on_kinematics_port_released)
        except Exception:
            pass

        if self._serial_suspendido and self._robot_service is not None:
            # Espera breve de seguridad
            QThread.msleep(200)
            self._robot_service.resume_serial()
        self._serial_suspendido = False

        self._set_inputs_enabled(False)

    def execute_kinematics(self):

        if not self._mode_active:
            return

        if not self.kinematics_worker.isRunning():
            return

        coords = self.kinematics_widget.get_coordinates()
        gains = self.kinematics_widget.get_pid_gains()
        targets = self.kinematics_widget.get_pid_targets()
        self.kinematics_worker.set_pid_gains(
            gains["kp"], gains["ki"], gains["kd"])
        self.kinematics_worker.set_pid_targets(
            targets["robot"], targets["sim"])

        tx_raw = coords['x']
        ty_raw = coords['y']
        tz_raw = coords['z']

        tx = tx_raw + 110
        ty = ty_raw
        tz = tz_raw
        tz = corregir_z(tx, ty, tz)
        tx, ty = corregir_xy(tx, ty)

        self._set_inputs_enabled(False)
        self.kinematics_widget.set_control_state("running")

        # Asegurar que el worker no esté pausado antes de enviar la tarea
        self.kinematics_worker.resume()

        self.kinematics_worker.execute_target(tx, ty, tz)

    @pyqtSlot(int)
    def _on_claw_changed(self, value):
        self._claw_mm = value
        if self._mode_active:
            self.kinematics_worker.set_claw_value(value)

    @pyqtSlot(str)
    def _on_status_changed(self, message):
        print(f"[Cinematica] {message}")

    @pyqtSlot()
    def _on_movement_finished(self):
        if self._mode_active:
            self._set_inputs_enabled(True)
            self.kinematics_widget.set_control_state("idle")

    @pyqtSlot()
    def _on_input_enabled(self):
        if self._mode_active:
            self._set_inputs_enabled(True)
            self.kinematics_widget.set_control_state("idle")

    @pyqtSlot(object)
    def _on_global_mode_changed(self, mode):
        if mode != Modes.KINEMATIC and self._mode_active:
            self.exit_kinematics_mode()

    def _on_restart(self):
        # 1. Bloquear inputs inmediatamente
        self._set_inputs_enabled(False)
        self.kinematics_widget.set_control_state("homing")

        # Resetear valor de garra
        self._claw_mm = 30
        self.kinematics_widget.reset_claw_value()

        # 2. Resetear el worker (abortar PID y limpiar cola)
        self.kinematics_worker.reset_state()
        self.kinematics_worker.resume()

        # 3. Resetear gráfica
        if self._graph_controller:
            self._graph_controller.reset_cartesian_plot()

        # 4. Enviar HOME (la señal input_enabled del worker re-habilitará la UI)
        self.kinematics_worker.send_home()

    def _set_inputs_enabled(self, enabled):
        self.kinematics_widget.coordinates_button.setEnabled(enabled)
        self.kinematics_widget.set_inputs_enabled(enabled)

    def get_widget(self):
        return self.kinematics_widget

    def get_worker(self):
        return self.kinematics_worker

    def cleanup(self):
        self.exit_kinematics_mode()
        if hasattr(self, 'kinematics_worker') and self.kinematics_worker:
            self.kinematics_worker.wait(3000)
