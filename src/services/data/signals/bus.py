"""
Bus de señales global del sistema (SignalBus).

Sustituye a los antiguos gestores de señales por dominio (los 12 archivos
SignalManager). Toda la comunicación entre threads (UI, workers, simulación,
hardware) pasa por una única clase con un único módulo-instancia:

    from src.services.data.signals import bus
    bus.sim_update_robot_signal.emit(posiciones)
    bus.cfg_request_change("settings.json", ["camera", "view"], valor)

Cada señal se declara con un prefijo de dominio para preservar la semántica
original: los dominios que compartían la clase base (_SignalManager) tenían
señales INDEPENDIENTES por instancia, de modo que emitir en un dominio no
alcanzaba a los suscriptores de otro. Los prefijos mantienen ese aislamiento.

Prefijos de dominio:
    sld_  control manual por sliders (ex SlidersSignalManager)
    sim_  simulación 3D / PyBullet (ex SimulationSignalManager)
    phy_  robot físico / serial (ex PhysicalSignalManager)
    kin_  control cartesiano (ex KinematicsSignalManager)
    cam_  cámara y visión (ex CameraSignalManager)
    pp_   pick and place (ex PickPlaceSignalManager)
    cfg_  configuración (ex ConfigSignalManager)
    search_  búsqueda visual (ex SearchSignalManager)
    theme_   tema visual (ex ThemeSignalManager)

La instancia ``bus`` se crea a nivel de módulo y es única para todo el
proceso. Los métodos de estado (config, búsqueda, pick and place, tema) se
incluyen aquí con su prefijo; el estado dependiente de la configuración se
inicializa perezosamente en el primer acceso, igual que hacían los antiguos
singletons creados bajo demanda.
"""

import threading

from PyQt6.QtCore import pyqtSignal, QObject
from PyQt6.QtGui import QIcon

from src.services.data.enums import Modes

__all__ = ["SignalBus", "bus"]


class SignalBus(QObject):
    """Bus único de señales y estado compartido del sistema.

    Sustituye a los 12 SignalManager. Ver docstring del módulo para la
    convención de prefijos de dominio.
    """

    # ------------------------------------------------------------------
    # Dominio SLIDERS (ex SlidersSignalManager; señales heredadas de base)
    # ------------------------------------------------------------------
    sld_get_data_signal = pyqtSignal()
    sld_sensor_position_signal = pyqtSignal(list)
    sld_model_position_signal = pyqtSignal(list, dict)
    sld_update_robot_signal = pyqtSignal(list)
    sld_update_graph_signal = pyqtSignal(list)
    sld_update_pybullet_signal = pyqtSignal(list)
    sld_update_target_signal = pyqtSignal(list)
    sld_change_mode_signal = pyqtSignal(object)

    # ------------------------------------------------------------------
    # Dominio SIMULACIÓN (ex SimulationSignalManager)
    # ------------------------------------------------------------------
    sim_get_data_signal = pyqtSignal()
    sim_sensor_position_signal = pyqtSignal(list)
    sim_model_position_signal = pyqtSignal(list, dict)
    sim_update_robot_signal = pyqtSignal(list)
    sim_update_graph_signal = pyqtSignal(list)
    sim_update_pybullet_signal = pyqtSignal(list)
    sim_update_target_signal = pyqtSignal(list)
    sim_change_mode_signal = pyqtSignal(object)
    # Propias del dominio de simulación
    sim_sphere_pos_from_camera = pyqtSignal(dict)
    sim_sphere_pos_from_pybullet = pyqtSignal(dict)
    sim_clear_spheres = pyqtSignal()
    sim_release_sphere = pyqtSignal(str)
    sim_reattach_sphere = pyqtSignal(str)
    sim_sphere_radius_changed = pyqtSignal(float)
    sim_start_simulation = pyqtSignal()
    sim_pause_simulation = pyqtSignal(bool)
    sim_resume_simulation = pyqtSignal()
    sim_stop_simulation = pyqtSignal()
    sim_update_robot_from_kinematics = pyqtSignal(list)
    sim_update_robot_from_sliders = pyqtSignal(list)
    sim_update_robot_from_simulation = pyqtSignal(list)
    sim_start_request = pyqtSignal()
    sim_pause_request = pyqtSignal(bool)
    sim_stop_request = pyqtSignal()

    # ------------------------------------------------------------------
    # Dominio FÍSICO / hardware (ex PhysicalSignalManager)
    # ------------------------------------------------------------------
    phy_get_data_signal = pyqtSignal()
    phy_sensor_position_signal = pyqtSignal(list)
    phy_model_position_signal = pyqtSignal(list, dict)
    phy_update_robot_signal = pyqtSignal(list)
    phy_update_graph_signal = pyqtSignal(list)
    phy_update_pybullet_signal = pyqtSignal(list)
    phy_update_target_signal = pyqtSignal(list)
    phy_change_mode_signal = pyqtSignal(object)
    # Propias del dominio físico
    phy_send_to_robot = pyqtSignal(list)
    phy_data_received = pyqtSignal(list, list)
    phy_start_service = pyqtSignal()
    phy_stop_service = pyqtSignal()
    phy_pause_service = pyqtSignal()
    phy_resume_service = pyqtSignal()
    phy_start_request = pyqtSignal()
    phy_stop_request = pyqtSignal()

    # ------------------------------------------------------------------
    # Dominio CINEMÁTICA (ex KinematicsSignalManager)
    # ------------------------------------------------------------------
    kin_get_data_signal = pyqtSignal()
    kin_sensor_position_signal = pyqtSignal(list)
    kin_model_position_signal = pyqtSignal(list, dict)
    kin_update_robot_signal = pyqtSignal(list)
    kin_update_graph_signal = pyqtSignal(list)
    kin_update_pybullet_signal = pyqtSignal(list)
    kin_update_target_signal = pyqtSignal(list)
    kin_change_mode_signal = pyqtSignal(object)
    # Propias del dominio de cinemática
    kin_inverse_kinematics_requested = pyqtSignal(dict)
    kin_inverse_kinematics_ready = pyqtSignal(dict)

    # ------------------------------------------------------------------
    # Dominio CÁMARA / visión (ex CameraSignalManager)
    # ------------------------------------------------------------------
    cam_available_cameras = pyqtSignal(list)
    cam_charuco_done = pyqtSignal(int, object)
    cam_circles_done = pyqtSignal(int, object)
    cam_pose_done = pyqtSignal(int, object)
    cam_spheres_detected_2d = pyqtSignal(dict)
    cam_poses_from_camera = pyqtSignal(dict)
    cam_clear_spheres_request = pyqtSignal()

    # ------------------------------------------------------------------
    # Dominio PICK AND PLACE (ex PickPlaceSignalManager)
    # ------------------------------------------------------------------
    pp_poses_from_camera = pyqtSignal(dict)
    pp_spheres_detected_2d = pyqtSignal(dict)
    pp_sphere_selected = pyqtSignal(str)
    pp_place_requested = pyqtSignal(dict)
    pp_pick_requested = pyqtSignal(str)
    pp_inverse_kinematics_requested = pyqtSignal(dict)
    pp_inverse_kinematics_ready = pyqtSignal(dict)
    pp_target_reached = pyqtSignal(list)
    pp_state_changed = pyqtSignal(bool)
    pp_change_mode_signal = pyqtSignal(object)
    pp_pick_place_running_changed = pyqtSignal(bool)
    pp_release_sphere_request = pyqtSignal(str)
    pp_reattach_sphere_request = pyqtSignal(str)
    pp_clear_spheres_request = pyqtSignal()
    # Petición de (des)activar la búsqueda de esferas en la cámara.
    # Sender PickAndPlaceController, receiver DataController.
    pp_search_circle_request = pyqtSignal(bool)

    # ------------------------------------------------------------------
    # Dominio CONFIGURACIÓN (ex ConfigSignalManager)
    # ------------------------------------------------------------------
    cfg_config_updated = pyqtSignal(str, list, object)
    cfg_change_requested = pyqtSignal(str, list, object)

    # ------------------------------------------------------------------
    # Dominio BÚSQUEDA VISUAL (ex SearchSignalManager)
    # ------------------------------------------------------------------
    search_charuco_changed = pyqtSignal(bool)
    search_circle_changed = pyqtSignal(bool)

    # ------------------------------------------------------------------
    # Dominio TEMA (ex ThemeSignalManager)
    # ------------------------------------------------------------------
    theme_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        # --- Configuración (ex ConfigSignalManager) -------------------
        self._cfg_cache = {}

        # --- Físico (ex PhysicalSignalManager.is_connected) -----------
        self.phy_is_connected = False

        # --- Simulación (estado del gemelo PyBullet) ------------------
        self._sim_running = False

        # --- Pick and place (ex PickPlaceSignalManager) ---------------
        self._pp_state_ready = False
        self._pp_active = False
        self._pp_running = False

        # --- Búsqueda visual (ex SearchSignalManager) -----------------
        self._search_state_ready = False
        self._search_lock = threading.RLock()
        self._search_charuco = False
        self._search_circle = False

        # --- Tema (ex ThemeSignalManager) -----------------------------
        self._current_theme = None
        self._toolbar_icons = {
            'sim_view':      ('icons:armView_d.png',       'icons:armView_l.png'),
            'camera_view':   ('icons:cameraView_d.png',    'icons:cameraView_l.png'),
            'graphs_view':   ('icons:graphView_d.png',     'icons:graphView_l.png'),
            'controls_view': ('icons:controlsView_d.png',  'icons:controlsView_l.png'),
            'charuco':       ('icons:gridSearch_d.png',    'icons:gridSearch_l.png'),
            'sphere':        ('icons:geometrySearch_d.png','icons:geometrySearch_l.png'),
            'start':         ('icons:play_d.png',          'icons:play_l.png'),
            'pause':         ('icons:pause_d.png',         'icons:pause_l.png'),
            'stop':          ('icons:stop_d.png',          'icons:stop_l.png'),
            'reset':         ('icons:refresh_d.png',       'icons:refresh_l.png'),
            'minimize':      ('icons:minimize_d.png',      'icons:minimize_l.png'),
            'restore':       ('icons:restore_down_d.png',  'icons:restore_down_l.png'),
            'close':         ('icons:close_d.svg',         'icons:close_l.svg'),
        }
        self._icon_cache = {}

    # ==================================================================
    # Configuración (ex ConfigSignalManager)
    # ==================================================================

    def cfg_set_all_config(self, filename: str, data: dict):
        """Puebla el caché interno con la configuración completa de un archivo.

        Args:
            filename (str): Nombre del archivo (e.g. 'settings.json').
            data (dict): Contenido completo del archivo.
        """
        self._cfg_cache[filename] = data

    def cfg_get_param(self, filename: str, *keys: str, default=None):
        """Obtiene un parámetro del caché de forma segura.

        Args:
            filename (str): Archivo donde buscar.
            *keys (str): Secuencia de llaves anidadas.
            default (any, optional): Valor a retornar si no existe.

        Returns:
            any: Valor encontrado o default.
        """
        data = self._cfg_cache.get(filename)
        if data is None:
            return default

        temp = data
        for key in keys:
            if isinstance(temp, dict) and key in temp:
                temp = temp[key]
            else:
                return default
        return temp

    def cfg_update_param(self, filename: str, keys: list, value: object, notify: bool = True):
        """Actualiza un parámetro en el caché local.

        Args:
            filename (str): Archivo a modificar.
            keys (list): Lista de llaves anidadas.
            value (object): Nuevo valor.
            notify (bool): Si se debe emitir la señal cfg_config_updated.
        """
        if filename not in self._cfg_cache:
            self._cfg_cache[filename] = {}

        data = self._cfg_cache[filename]
        target = data
        for key in keys[:-1]:
            target = target.setdefault(key, {})

        target[keys[-1]] = value

        if notify:
            self.cfg_config_updated.emit(filename, keys, value)

    def cfg_request_change(self, filename: str, keys: list, value: object):
        """Solicita un cambio persistente en la configuración.

        Args:
            filename (str): Archivo a modificar.
            keys (list): Llaves anidadas.
            value (object): Nuevo valor.
        """
        self.cfg_change_requested.emit(filename, keys, value)

    # ==================================================================
    # Simulación (estado del gemelo PyBullet)
    # ==================================================================

    def sim_set_running(self, running: bool):
        """Establece el estado de ejecución del gemelo físico (PyBullet).

        Estado simple consultado por los workers de control para decidir
        si el gemelo es una planta disponible; el ciclo de vida se comanda
        con las señales sim_start/pause/stop_simulation.

        Args:
            running (bool): True si la simulación está activa.
        """
        self._sim_running = running

    def sim_is_running(self) -> bool:
        """Retorna True si el gemelo físico (PyBullet) está activo.

        Returns:
            bool: Estado de ejecución de la simulación.
        """
        return self._sim_running

    # ==================================================================
    # Pick and place (ex PickPlaceSignalManager)
    # ==================================================================

    def _pp_ensure_state(self):
        """Inicializa perezosamente el estado desde la configuración."""
        if not self._pp_state_ready:
            self._pp_active = bool(self.cfg_get_param(
                "settings.json", "mode", "pick_place", default=False))
            self._pp_state_ready = True

    def pp_get_state(self) -> bool:
        """Retorna el estado del modo pick and place.

        Returns:
            bool: estado del modo.
        """
        self._pp_ensure_state()
        return self._pp_active

    def pp_set_state(self, state: bool):
        """Establece nuevo estado del modo pick and place.

        Args:
            state (bool): nuevo estado del modo.
        """
        self._pp_ensure_state()
        self._pp_active = state
        self.pp_state_changed.emit(state)
        if state:
            self.pp_change_mode_signal.emit(Modes.PICK_PLACE)
        else:
            self.pp_change_mode_signal.emit(Modes.SLIDERS)

    def pp_is_running(self) -> bool:
        """Retorna True si una secuencia de pick and place está en ejecución."""
        return self._pp_running

    def pp_set_running(self, running: bool):
        """Establece el estado de ejecución de la secuencia.

        Args:
            running (bool): True si la secuencia está activa.
        """
        self._pp_running = running
        self.pp_pick_place_running_changed.emit(running)

    # ==================================================================
    # Búsqueda visual (ex SearchSignalManager)
    # ==================================================================

    def _search_ensure_state(self):
        """Inicializa perezosamente el estado desde la configuración."""
        if not self._search_state_ready:
            with self._search_lock:
                if self._search_state_ready:
                    return
                state = self.cfg_get_param("settings.json", "camera", default={})
                self._search_charuco = state.get("charuco", False)
                self._search_circle = state.get("circle", False)
                self._search_state_ready = True

    def search_set_charuco(self, checked: bool):
        """Activa o desactiva la búsqueda de tablero ChArUco.

        Args:
            checked (bool): True para activar.
        """
        self._search_ensure_state()
        with self._search_lock:
            if self._search_charuco == checked:
                return
            self._search_charuco = checked
            self.cfg_request_change(
                "settings.json", ["camera", "charuco"], checked)

        # Emitir fuera del lock para evitar deadlocks con la UI
        self.search_charuco_changed.emit(checked)

    def search_set_circle(self, checked: bool):
        """Activa o desactiva la búsqueda de esferas de color.

        Args:
            checked (bool): True para activar.
        """
        self._search_ensure_state()
        with self._search_lock:
            if self._search_circle == checked:
                return
            self._search_circle = checked
            self.cfg_request_change(
                "settings.json", ["camera", "circle"], checked)

        # Emitir fuera del lock
        self.search_circle_changed.emit(checked)

    def search_get_state(self) -> tuple:
        """Obtiene el estado actual de ambas búsquedas.

        Returns:
            tuple: (charuco_activo, circle_activa).
        """
        self._search_ensure_state()
        with self._search_lock:
            return self._search_charuco, self._search_circle

    # ==================================================================
    # Tema (ex ThemeSignalManager)
    # ==================================================================

    def theme_emit_change(self, dark_t: bool):
        """Emite la señal de cambio de tema a todos los suscriptores.

        Args:
            dark_t (bool): True para tema oscuro.
        """
        self.theme_changed.emit(dark_t)

    def theme_set_current(self, theme):
        """Almacena el tema actual.

        Args:
            theme: Identificador del tema.
        """
        self._current_theme = theme

    def theme_get_current(self):
        """Obtiene el tema actual almacenado.

        Returns:
            Tema actual o None.
        """
        return self._current_theme

    def theme_get_toolbar_icon(self, name: str, is_dark: bool) -> QIcon:
        """Obtiene el icono de toolbar para el tema indicado, con cache.

        Args:
            name (str): Nombre de la entrada en _toolbar_icons.
            is_dark (bool): True para icono oscuro (_d), False para claro (_l).

        Returns:
            QIcon: Icono cacheado o recién creado.
        """
        key = (name, is_dark)
        if key not in self._icon_cache:
            dark_path, light_path = self._toolbar_icons[name]
            path = dark_path if is_dark else light_path
            self._icon_cache[key] = QIcon(path)
        return self._icon_cache[key]


# Instancia única del bus para todo el proceso.
bus = SignalBus()
