"""
Manejadores de acciones de la ventana principal (MainWindow).

Sustituye al antiguo MainActionsMixin por funciones planas que operan
sobre la ventana recibida. Implementa las acciones de control de flujo
(start/pause/stop/reset), la visibilidad de paneles (cámara, modelo 3D,
gráficos, controles), los modos de control y la inicialización de las
ventanas de calibración. ``wire_main_actions`` conecta las acciones del
menú/toolbar con estos manejadores.
"""

from src.services.data.enums.types import Modes, NotificationType
from src.services.data.signals import bus
from src.services.robot.serial_manager import SerialPortManager
from src.services.ui.notification_manager import NotificationManager


# ---------------------------------------------------------------------------
# Flujo de ejecución (start / pause / stop / reset)
# ---------------------------------------------------------------------------

def start(window):
    """Inicia la simulación, cámara, gráficos y servicios del robot.

    Deshabilita el botón de inicio y habilita pausa/stop.

    Args:
        window: MainWindow propietaria.
    """
    if window.hab_simulation:
        # Solicitar inicio vía DataController (Bus Global)
        bus.sim_start_request.emit()

    window.simulation_action.setEnabled(False)

    if window.camera_paused:
        window.camera_controller.resume_video()

    window.camera_controller.show_controls()
    window.com_submenu.setEnabled(True)

    if hasattr(window, 'kinematics_controller') and window.kinematics_controller and window.kinematics_controller._mode_active:
        window.kinematics_controller.get_worker().resume()

    if hasattr(window, 'pick_and_place_controller') and window.pick_and_place_controller:
        window.pick_and_place_controller.resume()

    if window.connected_to_robot:
        if window.stopped:
            bus.phy_start_request.emit()
        else:
            bus.phy_resume_service.emit()

    window.graph_controller.start()

    window.start_action.setEnabled(False)
    window.start_action.setChecked(True)
    window.pause_action.setEnabled(True)
    window.pause_action.setChecked(False)
    window.stop_action.setEnabled(True)
    window.stop_action.setChecked(False)

    window.stopped = False


def pause(window):
    """Pausa la simulación y gráficos (la cámara sigue activa).

    Detiene el servicio del robot si está conectado.
    Habilita el botón de inicio y deshabilita pausa.

    Args:
        window: MainWindow propietaria.
    """
    bus.sim_pause_request.emit(True)

    window.graph_controller.pause()

    # El RobotController manejará la pausa si escucha el bus, o podemos enviar stop
    if window.connected_to_robot:
        bus.phy_pause_service.emit()

    if hasattr(window, 'kinematics_controller') and window.kinematics_controller:
        window.kinematics_controller.get_worker().pause()

    if hasattr(window, 'pick_and_place_controller') and window.pick_and_place_controller:
        window.pick_and_place_controller.pause()

    window.pause_action.setEnabled(False)
    window.pause_action.setChecked(True)
    window.start_action.setEnabled(True)
    window.start_action.setChecked(False)


def stop(window):
    """Detiene la simulación, cámara y gráficos.

    Restaura el estado de reposo de la interfaz.

    Args:
        window: MainWindow propietaria.
    """
    if window.hab_simulation:
        bus.sim_stop_request.emit()

    window.simulation_action.setEnabled(True)
    window.model_action.setEnabled(True)

    window.camera_controller.stop_video()

    window.graph_controller.stop()

    if window.connected_to_robot:
        bus.phy_stop_request.emit()

    window.start_action.setEnabled(True)
    window.start_action.setChecked(False)
    window.pause_action.setEnabled(False)
    window.pause_action.setChecked(False)
    window.stop_action.setEnabled(False)
    window.stop_action.setChecked(True)

    window.stopped = True


def perform_system_reset(window):
    """Método auxiliar para resetear componentes de forma unificada.

    Args:
        window: MainWindow propietaria.
    """
    if hasattr(window, 'kinematics_controller') and window.kinematics_controller:
        window.kinematics_controller.kinematics_worker.reset_state()
    if hasattr(window, 'pick_and_place_controller') and window.pick_and_place_controller:
        window.pick_and_place_controller.stop_and_reset()

    if hasattr(window, 'graph_controller') and window.graph_controller:
        window.graph_controller.reset_cartesian_plot()


def reset(window):
    """Reinicia los controles según el modo activo.

    Args:
        window: MainWindow propietaria.
    """
    # 1. Reset Cinemática
    if hasattr(window, 'kinematics_controller') and window.kinematics_controller and window.kinematics_controller._mode_active:
        window.kinematics_controller._on_restart()

    # 2. Reset Pick and Place
    elif hasattr(window, 'pick_and_place_controller') and window.pick_and_place_controller and window.pick_and_place_controller.signal_manager.pp_get_state():
        window.pick_and_place_controller.restart_sequence()

    # 3. Reset Sliders (modo por defecto)
    else:
        window.sliders_controller.reset_controls()
        if hasattr(window, 'graph_controller') and window.graph_controller:
            window.graph_controller.reset_cartesian_plot()


# ---------------------------------------------------------------------------
# Visibilidad de paneles (docks)
# ---------------------------------------------------------------------------

def toggle_camera_dock(window, checked: bool):
    """Alterna la visibilidad del dock de la cámara.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar el panel.
    """
    window.cameraDock.toggleView(checked)
    bus.cfg_request_change(
        "settings.json", ["content", "camera"], checked)


def toggle_model_dock(window, checked: bool):
    """Alterna la visibilidad del dock del modelo 3D.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar el panel.
    """
    window.modelDock.toggleView(checked)
    bus.cfg_request_change(
        "settings.json", ["content", "model"], checked)


def toggle_graphs_dock(window, checked: bool):
    """Alterna la visibilidad del dock de gráficos.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar el panel.
    """
    window.graphsDock.toggleView(checked)
    bus.cfg_request_change(
        "settings.json", ["content", "graphs"], checked)


def toggle_controls_dock(window, checked: bool):
    """Alterna la visibilidad del dock de controles.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar el panel.
    """
    window.controlsDock.toggleView(checked)
    bus.cfg_request_change(
        "settings.json", ["content", "controls"], checked)


# ---------------------------------------------------------------------------
# Ventanas de calibración
# ---------------------------------------------------------------------------

def initiate_camera_calibration(window):
    """Inicializa la ventana de calibración de cámara.

    Detiene la cámara de la interfaz principal si está corriendo.
    Importación local para evitar círculos.

    Args:
        window: MainWindow propietaria.
    """
    from src.features.calibration import CameraCalibrationWindow

    window.camera_controller.stop_video()

    window.calibration_window = CameraCalibrationWindow()
    window.calibration_window.show()


def initiate_color_calibration(window):
    """Inicializa la ventana de calibración de colores.

    Detiene la cámara de la interfaz principal si está corriendo.
    Importación local para evitar círculos.

    Args:
        window: MainWindow propietaria.
    """
    from src.features.color import ColorWindow

    window.camera_controller.stop_video()

    window.color_window = ColorWindow()
    window.color_window.show()


# ---------------------------------------------------------------------------
# Ajustes de simulación y búsqueda visual
# ---------------------------------------------------------------------------

def toggle_simulation_activation(window, checked: bool):
    """Habilita o deshabilita la activación de la simulación física.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para activar la simulación.
    """
    window.hab_simulation = checked
    bus.cfg_request_change("settings.json", ["simulation",
                                             "activated"], checked)


def toggle_shadows(window, checked: bool):
    """Alterna las sombras de la escena 3D (persistencia en config)."""
    bus.cfg_request_change(
        "settings.json", ["simulation", "shadows"], checked)


def toggle_grid(window, checked: bool):
    """Alterna la malla infinita (persistencia en config)."""
    bus.cfg_request_change(
        "settings.json", ["simulation", "grid"], checked)


def toggle_axes(window, checked: bool):
    """Alterna los ejes de coordenadas (persistencia en config)."""
    bus.cfg_request_change(
        "settings.json", ["simulation", "axes"], checked)


def toggle_labels(window, checked: bool):
    """Alterna las etiquetas de juntas (persistencia en config)."""
    bus.cfg_request_change(
        "settings.json", ["simulation", "labels"], checked)


def toggle_aa(window, checked: bool):
    """Alterna el antialiasing MSAA (persistencia en config)."""
    bus.cfg_request_change(
        "settings.json", ["simulation", "aa"], checked)


def toggle_charuco_search(window, checked: bool):
    """Alterna la búsqueda de tablero ChArUco en la cámara.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para activar la detección.
    """
    bus.search_set_charuco(checked)


def toggle_sphere_search(window, checked: bool):
    """Alterna la búsqueda de esferas de color en la cámara.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para activar la detección.
    """
    bus.search_set_circle(checked)


# ---------------------------------------------------------------------------
# Modos de control (sliders / cinemática / pick and place)
# ---------------------------------------------------------------------------

def toggle_sliders_controls(window, checked: bool):
    """Alterna la visibilidad del panel de control manual (sliders).

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar sliders.
    """
    if checked:
        # Reseteamos otros modos antes de activar sliders
        if hasattr(window, 'kinematics_controller') and window.kinematics_controller:
            window.kinematics_controller.exit_kinematics_mode()
            SerialPortManager.get_instance().release_access("KinematicsWorker")
        if hasattr(window, 'pick_and_place_controller') and window.pick_and_place_controller:
            window.pick_and_place_controller.stop_and_reset()
            SerialPortManager.get_instance().release_access("PickAndPlaceWorker")

        # Reanudar RobotWorker si está conectado
        if window.connected_to_robot and hasattr(window, 'robot_service'):
            window.robot_service.resume_serial()

        window.sliders_controller.get_widget().show()
        window.kinematics_controller.get_widget().set_vertical_layout()
        window.graph_controller.set_graph_mode(True)
        bus.sld_change_mode_signal.emit(Modes.SLIDERS)

        # Sincronización a HOME
        home = [0, 0, 0, 0, 0, 0]
        # 1. Sliders UI
        window.sliders_controller.set_external_values(home)
        # 2. 3D Model
        bus.sim_update_robot_from_sliders.emit(home)
        # 3. Physical Robot
        if window.connected_to_robot:
            bus.phy_update_target_signal.emit(home)
    else:
        window.sliders_controller.get_widget().hide()
        window.kinematics_controller.get_widget().set_horizontal_layout()
        # No liberamos RobotWorker aquí, ya que los sliders podrían seguir necesitándolo
    bus.cfg_request_change(
        'settings.json', ["mode", 'sliders'], checked)


def toggle_kinematics_controls(window, checked: bool):
    """Alterna la visibilidad del panel de control cinemático.

    Al activar el modo cinemático:
    - Verifica conexión del robot.
    - Envía HOME directo y espera 2.5s.
    - Habilita la entrada de coordenadas.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar cinemática.
    """
    if checked:
        # Reseteamos otros modos antes de activar cinemática
        perform_system_reset(window)
        SerialPortManager.get_instance().release_access("RobotWorker")
        SerialPortManager.get_instance().release_access("PickAndPlaceWorker")

        window.kinematics_controller.get_widget().show()
        window.kinematics_controller.enter_kinematics_mode()
        window.graph_controller.set_graph_mode(False)
        bus.kin_change_mode_signal.emit(Modes.KINEMATIC)
    else:
        window.kinematics_controller.get_widget().hide()
        window.kinematics_controller.exit_kinematics_mode()
        SerialPortManager.get_instance().release_access("KinematicsWorker")
    bus.cfg_request_change(
        'settings.json', ["mode", 'kinematics'], checked)


def toggle_pick_place_controls(window, checked: bool):
    """Alterna la visibilidad del panel de control pick and place.

    Args:
        window: MainWindow propietaria.
        checked (bool): True para mostrar el panel de controles.
    """
    if checked and not window.connected_to_robot:
        NotificationManager.get_instance().notify(
            "Para usar el modo Pick and Place debe conectar el robot previamente desde el menú Robot > Puerto.",
            NotificationType.DIALOG_WARNING
        )
        return

    if checked:
        # Reseteamos otros modos antes de activar pick and place
        perform_system_reset(window)
        SerialPortManager.get_instance().release_access("RobotWorker")
        SerialPortManager.get_instance().release_access("KinematicsWorker")

    bus.pp_set_state(checked)

    if not checked:
        SerialPortManager.get_instance().release_access("PickAndPlaceWorker")

    # Ya no forzamos la ocultación del panel de controles aquí.
    # El controlador P&P se encarga de ajustar el contenido (PID only).
    bus.cfg_request_change(
        'settings.json', ["mode", 'pick_place'], checked)


# ---------------------------------------------------------------------------
# Conexión del robot
# ---------------------------------------------------------------------------

def connect_robot(window):
    """Inicia la conexión con el microcontrolador en el puerto COM seleccionado.

    Envía posición home [0,0,0,0,0,0] al robot y configura el target global.

    Args:
        window: MainWindow propietaria.
    """
    if not window.com:
        print("Error: Dispositivo no detectado")
        return
    window.connected_to_robot = True
    from src.main_window import layout_builder
    layout_builder.init_openbotv(window, window.com)
    window.com_connected_label.setText(window.com)

    home = [0, 0, 0, 0, 0, 0]
    window.sliders_controller.set_external_values(home)
    bus.phy_update_target_signal.emit(home)


# ---------------------------------------------------------------------------
# Cableado de acciones
# ---------------------------------------------------------------------------

def wire_main_actions(window):
    """Configura todas las conexiones de señales y slots de la interfaz.

    Enlaza botones, acciones de menú y eventos con los manejadores de
    este módulo (ex setup_connections de MainWindow).

    Args:
        window: MainWindow propietaria de las acciones.
    """
    # Visibilidad de ventanas/paneles
    if hasattr(window, 'cameraBox'):
        if hasattr(window, 'camera_action'):
            window.camera_action.triggered.connect(
                lambda checked: toggle_camera_dock(window, checked))
    if hasattr(window, 'modelBox'):
        if hasattr(window, 'model_action'):
            window.model_action.triggered.connect(
                lambda checked: toggle_model_dock(window, checked))
    if hasattr(window, 'graphs_action'):
        window.graphs_action.triggered.connect(
            lambda checked: toggle_graphs_dock(window, checked))
    if hasattr(window, 'controls_action'):
        window.controls_action.triggered.connect(
            lambda checked: toggle_controls_dock(window, checked))

    # Configuración de detección visual
    if hasattr(window, 'charuco_action'):
        window.charuco_action.toggled.connect(
            lambda checked: toggle_charuco_search(window, checked))
        bus.search_charuco_changed.connect(
            window.charuco_action.setChecked)
    if hasattr(window, 'sphere_action'):
        window.sphere_action.toggled.connect(
            lambda checked: toggle_sphere_search(window, checked))
        bus.search_circle_changed.connect(
            window.sphere_action.setChecked)
    if hasattr(window, 'camera_calibration_action'):
        window.camera_calibration_action.triggered.connect(
            lambda: initiate_camera_calibration(window))
    if hasattr(window, 'color_calibration_action'):
        window.color_calibration_action.triggered.connect(
            lambda: initiate_color_calibration(window))

    # Visibilidad y uso de controles (Manual/Cinematico)
    if hasattr(window, "sliders_action"):
        window.sliders_action.toggled.connect(
            lambda checked: toggle_sliders_controls(window, checked))
    if hasattr(window, "kinematics_action"):
        window.kinematics_action.toggled.connect(
            lambda checked: toggle_kinematics_controls(window, checked))
    if hasattr(window, "pick_place_action"):
        window.pick_place_action.toggled.connect(
            lambda checked: toggle_pick_place_controls(window, checked))

    # Sincronizacion Kinematics -> Sliders
    # (El worker de cinematica ahora opera de forma independiente
    # con su propio serial, por lo que esta conexion ya no es necesaria)

    # Botones de control de flujo (Start, Pause, Stop, Reset)
    if hasattr(window, 'start_action'):
        window.start_action.triggered.connect(lambda: start(window))
    if hasattr(window, 'pause_action'):
        window.pause_action.triggered.connect(lambda: pause(window))
    if hasattr(window, 'stop_action'):
        window.stop_action.triggered.connect(lambda: stop(window))
    if hasattr(window, 'reset_action'):
        window.reset_action.triggered.connect(lambda: reset(window))

    # Activación de simulación física
    if hasattr(window, 'simulation_action'):
        window.simulation_action.triggered.connect(
            lambda checked: toggle_simulation_activation(window, checked))
    if hasattr(window, 'shadows_action'):
        window.shadows_action.triggered.connect(
            lambda checked: toggle_shadows(window, checked))
    if hasattr(window, 'grid_action'):
        window.grid_action.triggered.connect(
            lambda checked: toggle_grid(window, checked))
    if hasattr(window, 'axes_action'):
        window.axes_action.triggered.connect(
            lambda checked: toggle_axes(window, checked))
    if hasattr(window, 'labels_action'):
        window.labels_action.triggered.connect(
            lambda checked: toggle_labels(window, checked))
    if hasattr(window, 'aa_action'):
        window.aa_action.triggered.connect(
            lambda checked: toggle_aa(window, checked))

    # Control de temas y conexión de hardware
    if hasattr(window, 'theme_action'):
        window.theme_action.triggered.connect(
            window.theme_manager.toggle_theme_event)

    if hasattr(window, 'connect_action'):
        window.connect_action.triggered.connect(
            lambda: connect_robot(window))
