"""
Módulo que actúa como controlador de flujo de datos centralizado (Cerebro).

Implementa la lógica de negocio y orquestación, actuando como un mediador reactivo
entre los controladores de interfaz (Features) y los controladores de ejecución
(Servicios de Simulación y Robot).

Reglas de Arquitectura:
    - Instancia única gestionada por MainWindow.
    - Se comunica con otros controladores exclusivamente vía el bus de señales.
    - No tiene acceso directo a Workers o Widgets ajenos.
"""

import math
from PyQt6.QtCore import QObject, pyqtSlot
from src.services.data import config_manager
from src.services.data.signals import bus
from src.services.data.enums import Modes, Units
from src.services.data.timers import global_timer
from src.services.data.utils import deg_to_rad, rad_to_deg, angulos_robotang, robotang_angulos


class DataController(QObject):
    """
    Orquestador central del flujo de datos del sistema.

    Mantiene el estado global del robot (Digital Twin) y coordina la 
    sincronización entre la entrada del usuario, la simulación física 
    y el hardware real.
    """

    _config_initialized = False

    def __init__(self) -> None:
        """
        Inicializa el DataController único.
        """
        super().__init__()
        self._mode = Modes.SLIDERS
        self._units = Units.RAD
        self._target_data = None
        self._last_feedback = None
        self._TARGET_TOLERANCE = 3.0

        # Gestores de señales globales (Bus Inter-Controller)
        self.sim_signals = bus
        self.phys_signals = bus
        self.pick_signals = bus
        self.config_signals = bus
        self.cam_signals = bus
        self.sliders_signals = bus
        self.kin_signals = bus
        self.search_signals = bus

        # Inicialización de configuración (una sola vez)
        if not DataController._config_initialized:
            self._load_initial_config()
            bus.cfg_change_requested.connect(
                self._on_config_change_requested)
            DataController._config_initialized = True

        self._setup_global_connections()

        # Temporizador de sincronización centralizado
        self._sync_timer = global_timer
        self._sync_timer.start()
        self._sync_timer.sync_robot_tick.connect(self._handle_sync_tick)

    def _load_initial_config(self):
        """Carga la configuración persistente."""
        config_manager.init_config()
        for filename in config_manager.DEFAULTS.keys():
            data = config_manager.load(filename)
            bus.cfg_set_all_config(filename, data)

    @pyqtSlot(str, list, object)
    def _on_config_change_requested(self, filename: str, keys: list, value: object):
        """Persiste cambios de configuración."""
        config_manager.set_value(filename, keys, value)
        bus.cfg_update_param(filename, keys, value)

    def _setup_global_connections(self):
        """
        Configura el ruteo de señales entre controladores.
        """
        # --- Entradas desde Features (UI) ---
        # Escuchar actualizaciones de objetivo desde cualquier dominio
        bus.sim_update_target_signal.connect(
            self.update_target_positions)
        bus.phy_update_target_signal.connect(
            self.update_target_positions)
        bus.sld_update_target_signal.connect(
            self.update_target_positions)
        bus.kin_update_target_signal.connect(
            self.update_target_positions)

        # Escuchar cambios de modo
        bus.sim_change_mode_signal.connect(self.set_mode)
        bus.phy_change_mode_signal.connect(self.set_mode)
        bus.sld_change_mode_signal.connect(self.set_mode)
        bus.kin_change_mode_signal.connect(self.set_mode)
        bus.pp_change_mode_signal.connect(self.set_mode)

        # --- Feedback desde Servicios (Ejecucion) ---
        # Feedback de Simulación -> Actualización de UI
        bus.sim_sensor_position_signal.connect(
            self._on_simulation_feedback)
        bus.sim_model_position_signal.connect(self._on_model_feedback)

        # Feedback de Robot Físico -> Actualización de UI y Sincronización
        bus.phy_data_received.connect(self._on_physical_feedback)

        # Bridge Pick and Place -> Simulation
        bus.pp_release_sphere_request.connect(
            bus.sim_release_sphere.emit)
        bus.pp_reattach_sphere_request.connect(
            bus.sim_reattach_sphere.emit)

        # Bridge Pick and Place -> Search (control de búsqueda de esferas)
        bus.pp_search_circle_request.connect(
            bus.search_set_circle)

        # Bridge cinemática inversa: Pick and Place <-> Kinematics.
        # Ninguna feature conoce a la otra; el DataController media el diálogo.
        bus.pp_inverse_kinematics_requested.connect(
            bus.kin_inverse_kinematics_requested.emit)
        bus.kin_inverse_kinematics_ready.connect(
            bus.pp_inverse_kinematics_ready.emit)

        # Bridge Camera (vision) -> Simulation / Pick and Place
        bus.cam_clear_spheres_request.connect(
            bus.sim_clear_spheres.emit)
        bus.cam_poses_from_camera.connect(
            bus.sim_sphere_pos_from_camera.emit)
        bus.cam_poses_from_camera.connect(
            bus.pp_poses_from_camera.emit)
        bus.cam_spheres_detected_2d.connect(
            bus.pp_spheres_detected_2d.emit)

        # Bridge Simulation Control
        bus.sim_start_request.connect(
            bus.sim_start_simulation.emit)
        bus.sim_pause_request.connect(
            bus.sim_pause_simulation.emit)
        bus.sim_stop_request.connect(
            bus.sim_stop_simulation.emit)

        # Bridge Robot Control
        bus.phy_start_request.connect(
            bus.phy_start_service.emit)
        bus.phy_stop_request.connect(
            bus.phy_stop_service.emit)

    @pyqtSlot()
    def _handle_sync_tick(self):
        """Despacha posiciones y detecta llegada al objetivo."""
        if self._target_data is None:
            return

        # Si estamos en modo KINEMATIC, solo permitimos enviar al robot si es parte de una operacion P&P
        # (ej. el Worker emite comandos directos via action_request)
        if self._mode == Modes.KINEMATIC and not bus.pp_is_running():
            return
        
        if self._mode == Modes.PICK_PLACE and not bus.pp_is_running():
            # Si estamos en Pick & Place pero la secuencia no está corriendo, no enviar sync tick de sliders
            return

        data_rad = deg_to_rad(self._target_data)
        # print(f"[DEBUG DataController] _handle_sync_tick emitting update_pybullet_signal with target_data: {self._target_data}, mode: {self._mode}")
        bus.sim_update_pybullet_signal.emit(data_rad.tolist())

        if bus.phy_is_connected:
            robot_positions = list(angulos_robotang(*self._target_data))
            bus.phy_send_to_robot.emit(robot_positions)

        if self._last_feedback is not None:
            feedback = list(robotang_angulos(*self._last_feedback))
            if all(
                abs(self._target_data[i] - feedback[i])
                < self._TARGET_TOLERANCE
                for i in range(min(len(self._target_data),
                                   len(feedback)))
            ):
                bus.pp_target_reached.emit(feedback)
                self._last_feedback = None

    # --- Slots de Procesamiento Lógico ---

    @pyqtSlot(list)
    def update_target_positions(self, data: list):
        """Actualiza el buffer de estado central."""
        print(f"[DEBUG DataController] update_target_positions called with data: {data}, mode: {self._mode}")
        self._target_data = data

    @pyqtSlot(object)
    def set_mode(self, mode: Modes):
        """Cambia el modo de operación global."""
        print(f"[DEBUG DataController] set_mode called: switching from {self._mode} to {mode}")
        self._mode = mode

    @pyqtSlot(list)
    def _on_simulation_feedback(self, actual_positions: list):
        """Procesa feedback de la simulación para gráficos y target_reached."""
        pos_deg = rad_to_deg(actual_positions)
        bus.sim_update_graph_signal.emit(pos_deg)
        if not bus.phy_is_connected:
            self._last_feedback = pos_deg

    @pyqtSlot(list, dict)
    def _on_model_feedback(self, motor_positions, sphere_positions):
        """Procesa feedback de la simulación para el modelo 3D."""
        if self._mode not in (Modes.KINEMATIC, Modes.PICK_PLACE):
            pos_deg = rad_to_deg(motor_positions)
            # print(f"[DEBUG DataController] _on_model_feedback emitting update_robot_from_simulation with pos_deg: {pos_deg}")
            bus.sim_update_robot_from_simulation.emit(pos_deg)
        bus.sim_sphere_pos_from_pybullet.emit(sphere_positions)

    @pyqtSlot(list, list)
    def _on_physical_feedback(self, actual_positions: list, temperatures: list):
        """Procesa feedback del robot real para sincronizar sistema."""
        bus.phy_update_graph_signal.emit(actual_positions)
        self._last_feedback = actual_positions
