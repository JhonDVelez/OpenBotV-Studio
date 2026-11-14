"""
Construcción del layout e inicialización de módulos de MainWindow.

Sustituye al antiguo MainInitMixin por funciones planas que operan sobre
la ventana recibida. Crea los contenedores acoplables (docks de ADS),
inicializa los controladores de cámara, simulación, gráficos y controles,
y configura la toolbar lateral.
"""

from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QApplication, QGridLayout, QGroupBox, QHBoxLayout, QSizePolicy,
    QToolBar, QVBoxLayout, QWidget)
import PyQt6Ads as ads

from src.services.data.signals import bus
from src.services.robot import RobotController
from src.features.camera import CameraController
from src.features.sliders import SlidersController
from src.features.kinematics import KinematicsController
from src.features.pick_and_place import PickAndPlaceController
from src.features.simulation.simulation_controller import SimulationController
from src.features.graph import GraphController


def setup_ui(window):
    """Configura la UI dinámica basada en PyQt6-Ads.

    Crea los contenedores acoplables para el modelo, cámara, gráficas y
    controles. La gestión del layout reside exclusivamente en el
    CDockManager.

    Args:
        window: MainWindow propietaria (usa window.inner_window).
    """
    # Inicializar el Dock Manager. No establecemos Central Widget manualmente
    # para evitar conflictos que causan el borrado de objetos en C++.
    ads.CDockManager.setConfigFlag(
        ads.CDockManager.eConfigFlag.DisableTabTextEliding, True)
    ads.CDockManager.setConfigFlag(
        ads.CDockManager.eConfigFlag.AlwaysShowTabs, False)
    ads.CDockManager.setConfigFlag(
        ads.CDockManager.eConfigFlag.HideSingleCentralWidgetTitleBar, True)
    ads.CDockManager.setConfigFlag(
        ads.CDockManager.eConfigFlag.DockAreaDynamicTabsMenuButtonVisibility, True)

    window.dock_manager = ads.CDockManager(window.inner_window)
    window.dock_manager.setObjectName("dockManager")

    # Resolver el enum de áreas para la versión 4.5.0
    Area = ads.DockWidgetArea

    box_size_policy = QSizePolicy(
        QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    # 1. Dock del Modelo 3D
    window.modelDock = ads.CDockWidget("Visualización 3D")
    window.modelDock.setObjectName("modelDock")
    window.modelBox = QGroupBox(window.inner_window)
    window.modelBox.setTitle("")
    window.modelBox.setObjectName("modelBox")
    # Forzamos eliminación de bordes nativos también vía código
    window.modelBox.setFlat(True)
    window.modelBox.setSizePolicy(box_size_policy)
    window.modelDock.setWidget(window.modelBox)
    window.modelDock.setMinimumSizeHintMode(
        ads.CDockWidget.eMinimumSizeHintMode.MinimumSizeHintFromContent)
    window.dock_manager.addDockWidget(
        Area.LeftDockWidgetArea, window.modelDock)
    _configure_dock_title_bar(window.modelDock)

    # 2. Dock de la Cámara
    window.cameraDock = ads.CDockWidget("Cámara en Vivo")
    window.cameraDock.setObjectName("cameraDock")
    window.cameraBox = QGroupBox(window.inner_window)
    window.cameraBox.setTitle("")
    window.cameraBox.setObjectName("cameraBox")
    window.cameraBox.setFlat(True)
    window.cameraBox.setSizePolicy(box_size_policy)
    window.cameraDock.setWidget(
        window.cameraBox, ads.CDockWidget.eInsertMode.ForceNoScrollArea)
    window.cameraDock.setMinimumSizeHintMode(
        ads.CDockWidget.eMinimumSizeHintMode.MinimumSizeHintFromDockWidgetMinimumSize)
    window.dock_manager.addDockWidget(
        Area.BottomDockWidgetArea, window.cameraDock,
        window.modelDock.dockAreaWidget())
    _configure_dock_title_bar(window.cameraDock)

    # 3. Dock de Gráficas
    window.graphsDock = ads.CDockWidget("Telemetría y Gráficas")
    window.graphsDock.setObjectName("graphsDock")
    window.graphsBox = QGroupBox(window.inner_window)
    window.graphsBox.setTitle("")
    window.graphsBox.setObjectName("graphsBox")
    window.graphsBox.setFlat(True)
    window.graphsBox.setSizePolicy(box_size_policy)
    window.graphsDock.setWidget(window.graphsBox)
    window.graphsDock.setMinimumSizeHintMode(
        ads.CDockWidget.eMinimumSizeHintMode.MinimumSizeHintFromDockWidgetMinimumSize)
    window.dock_manager.addDockWidget(
        Area.RightDockWidgetArea, window.graphsDock)
    _configure_dock_title_bar(window.graphsDock)

    # 4. Dock de Controles
    window.controlsDock = ads.CDockWidget("Panel de Control")
    window.controlsDock.setObjectName("controlsDock")
    window.controlsBox = QGroupBox(window.inner_window)
    window.controlsBox.setTitle("")
    window.controlsBox.setObjectName("controlsBox")
    window.controlsBox.setFlat(True)
    window.controlsBox.setSizePolicy(box_size_policy)

    window.verticalLayout_2 = QVBoxLayout(window.controlsBox)
    window.verticalLayout_2.setSpacing(0)
    window.verticalLayout_2.setContentsMargins(0, 0, 0, 0)

    window.controlsDock.setWidget(
        window.controlsBox, ads.CDockWidget.eInsertMode.ForceNoScrollArea)
    window.controlsDock.setMinimumSizeHintMode(
        ads.CDockWidget.eMinimumSizeHintMode.MinimumSizeHintFromDockWidgetMinimumSize)
    window.dock_manager.addDockWidget(
        Area.BottomDockWidgetArea, window.controlsDock,
        window.graphsDock.dockAreaWidget())
    _configure_dock_title_bar(window.controlsDock)

    # Configurar proporciones iniciales (ADS lo maneja internamente tras el layout)
    for i, dock in window.dock_manager.dockWidgetsMap().items():
        _configure_dock_title_bar(dock)
        dock.topLevelChanged.connect(
            lambda floating, d=dock: _on_dock_floating_changed(d))


def _configure_dock_title_bar(dock):
    """Apply title bar margins and button object names to a dock widget."""
    area = dock.dockAreaWidget()
    if area is None:
        return
    title = area.titleBar()
    area.layout().setContentsMargins(0, 0, 0, 0)
    dock.layout().setContentsMargins(0, 0, 0, 0)
    undock_btn = title.button(ads.TitleBarButton.TitleBarButtonUndock)
    if undock_btn is not None:
        undock_btn.setObjectName("undockBtn")
        undock_btn.setVisible(True)
    close_btn = title.button(ads.TitleBarButton.TitleBarButtonClose)
    if close_btn is not None:
        close_btn.setObjectName("closeBtn")


def _on_dock_floating_changed(dock):
    """Re-apply title bar config when a dock is docked (floating=False) or undocked (floating=True)."""
    _configure_dock_title_bar(dock)


def init_camera(window):
    """Inicializa el módulo de cámara y lo agrega a su contenedor.

    Args:
        window: MainWindow propietaria.
    """
    if not window.cameraBox.layout():
        layout = QVBoxLayout(window.cameraBox)
        layout.setContentsMargins(0, 0, 0, 0)
        window.cameraBox.setLayout(layout)

    window.camera_controller = CameraController(window)
    window.cameraBox.layout().addWidget(
        window.camera_controller.get_widget())

    # Inicializar el controlador de Pick and Place asociado a la cámara
    window.pick_and_place_controller = PickAndPlaceController(
        window.camera_controller.get_widget()
    )

    window.camera_controller.status_changed.connect(
        window.camera_connected_label.setText)
    window.camera_controller.active_state_changed.connect(
        lambda active: window.camera_interval_submenu.setEnabled(active)
        if hasattr(window, 'camera_interval_submenu') else None
    )


def init_controls(window):
    """Inicializa los paneles de control (sliders y cinemática).

    Crea los controladores de sliders y cinemática, los agrega a un
    contenedor compartido y configura la visibilidad inicial.

    Args:
        window: MainWindow propietaria.
    """
    window.kinematics_controller = KinematicsController(window)
    window.sliders_controller = SlidersController(window)
    window.modes_widget = QWidget()

    if not window.modes_widget.layout():
        layout = QHBoxLayout(window.modes_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        window.modes_widget.setLayout(layout)

    window.modes_widget.layout().addWidget(
        window.sliders_controller.get_widget())
    window.modes_widget.layout().addWidget(
        window.kinematics_controller.get_widget())
    window.modes_widget.setMaximumSize(QSize(1050, 300))

    window.kinematics_controller.get_widget().hide()

    window.controlsBox.layout().addStretch(1)
    window.controlsBox.layout().addWidget(window.modes_widget)


def init_tool_bar(window):
    """Crea la barra de herramientas lateral con las acciones de la app.

    Incluye botones de visibilidad de paneles (modelo 3D, cámara,
    gráficos, controles), activación de búsqueda visual (ChArUco,
    esferas) y controles de flujo (start, pause, stop, reset).

    Args:
        window: MainWindow propietaria.
    """
    window.actions_toolbar = QToolBar()
    window.actions_toolbar.setObjectName("actions_toolbar")
    window.actions_toolbar.setOrientation(Qt.Orientation.Vertical)
    window.actions_toolbar.setIconSize(QSize(24, 24))
    window.actions_toolbar.setMinimumHeight(30)
    window.actions_toolbar.setMinimumWidth(30)
    window.actions_toolbar.setContentsMargins(0, 0, 0, 0)
    window.actions_toolbar.setMovable(True)
    window.actions_toolbar.setFloatable(True)

    window.toolbar_spacer = QWidget()
    window.toolbar_spacer.setSizePolicy(
        QSizePolicy.Policy.Expanding,
        QSizePolicy.Policy.Expanding
    )
    window.toolbar_spacer.setObjectName("toolbar_spacer")

    window.toolbar_signal_manager = bus
    is_dark = bus.theme_get_current() == Qt.ColorScheme.Dark

    window.model_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('sim_view', is_dark), "Vista de simulación")
    window.model_action.setObjectName("simulation_view_button")
    window.model_action.setStatusTip(
        "Mostrar/Ocultar el modelo 3D de la simulación")
    window.model_action.setCheckable(True)

    window.camera_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('camera_view', is_dark), "Vista de cámara")
    window.camera_action.setObjectName("camera_view_button")
    window.camera_action.setStatusTip("Mostrar/Ocultar la cámara")
    window.camera_action.setCheckable(True)

    window.graphs_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('graphs_view', is_dark), "Vista de gráficas")
    window.graphs_action.setObjectName("graph_view_button")
    window.graphs_action.setStatusTip("Mostrar/Ocultar las gráficas")
    window.graphs_action.setCheckable(True)

    window.controls_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('controls_view', is_dark), "Vista de controles")
    window.controls_action.setObjectName("controls_view_button")
    window.controls_action.setStatusTip("Mostrar/Ocultar los controles")
    window.controls_action.setCheckable(True)

    window.actions_toolbar.widgetForAction(window.model_action)
    window.actions_toolbar.widgetForAction(window.camera_action)
    window.actions_toolbar.widgetForAction(window.graphs_action)
    window.actions_toolbar.widgetForAction(window.controls_action)

    window.actions_toolbar.addSeparator()
    window.actions_toolbar.addWidget(window.toolbar_spacer)
    window.actions_toolbar.addSeparator()

    window.charuco_icon = bus.theme_get_toolbar_icon(
        'charuco', is_dark)
    window.sphere_icon = bus.theme_get_toolbar_icon(
        'sphere', is_dark)

    window.charuco_action = window.actions_toolbar.addAction(
        window.charuco_icon, "Tablero ChArUco")
    window.charuco_action.setObjectName("charuco_action")
    window.charuco_action.setStatusTip(
        "Activa/Desactiva la detección del tablero")
    window.charuco_action.setCheckable(True)

    window.sphere_action = window.actions_toolbar.addAction(
        window.sphere_icon, "Activar Objetos")
    window.sphere_action.setObjectName("sphere_action")
    window.sphere_action.setStatusTip(
        "Activa/Desactiva la detección de las esferas de colores")
    window.sphere_action.setCheckable(True)

    settings = bus.cfg_get_param("settings.json", default={})
    camera_config = settings.get("camera", {})
    if "charuco" in camera_config:
        window.charuco_action.setChecked(camera_config.get("charuco"))
    if "circle" in camera_config:
        window.sphere_action.setChecked(camera_config.get("circle"))

    window.actions_toolbar.addSeparator()

    window.start_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('start', is_dark), "Iniciar")
    window.start_action.setObjectName("start_button")
    window.start_action.setStatusTip("Iniciar la ejecución")
    window.start_action.setCheckable(True)

    window.pause_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('pause', is_dark), "Pausar")
    window.pause_action.setObjectName("pause_button")
    window.pause_action.setStatusTip("Pausar la ejecución")
    window.pause_action.setEnabled(False)
    window.pause_action.setCheckable(True)

    window.stop_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('stop', is_dark), "Detener")
    window.stop_action.setObjectName("stop_button")
    window.stop_action.setStatusTip("Detener la ejecución")
    window.stop_action.setEnabled(False)
    window.stop_action.setCheckable(True)
    window.stop_action.setChecked(True)

    window.reset_action = window.actions_toolbar.addAction(
        bus.theme_get_toolbar_icon('reset', is_dark), "Reiniciar")
    window.reset_action.setObjectName("reset_button")
    window.reset_action.setStatusTip("Reiniciar los valores")

    window.start_button = window.actions_toolbar.widgetForAction(
        window.start_action)
    window.pause_button = window.actions_toolbar.widgetForAction(
        window.pause_action)
    window.stop_button = window.actions_toolbar.widgetForAction(
        window.stop_action)
    window.reset_button = window.actions_toolbar.widgetForAction(
        window.reset_action)


def init_simulation(window):
    """Inicializa el módulo de simulación y lo agrega a su contenedor.

    Crea el SimulationController que integra la vista Quick3D con
    el motor de física PyBullet.

    Args:
        window: MainWindow propietaria.
    """
    if not window.modelBox.layout():
        window.sim_layout = QVBoxLayout(window.modelBox)
        window.sim_layout.setContentsMargins(0, 0, 0, 0)
        window.modelBox.setLayout(window.sim_layout)

    window.simulation_controller = SimulationController(
        window, window.preloaded_data, window.robot_id)
    window.sim_layout.addWidget(
        window.simulation_controller.get_simulation_widget())


def init_openbotv(window, com: str):
    """Inicializa la conexión con el microcontrolador vía puerto serie.

    Crea el RobotController para el dominio físico. El DataController
    central ya está orquestando las señales.

    Args:
        window: MainWindow propietaria.
        com (str): Nombre del puerto COM (e.g. 'COM3').
    """
    window.robot_service = RobotController(com)
    window.robot_service.start_service()

    window.kinematics_controller.set_robot_service(window.robot_service)

    window.connect_action.setEnabled(False)


def init_graphics(window):
    """Inicializa el módulo de gráficos de telemetría.

    Crea el GraphController con el servicio de cinemática para
    el cálculo de trayectorias cartesianas.

    Args:
        window: MainWindow propietaria.
    """
    if not window.graphsBox.layout():
        graph_layout = QVBoxLayout(window.graphsBox)
        graph_layout.setContentsMargins(0, 0, 0, 0)
        window.graphsBox.setLayout(graph_layout)

    window.graph_controller = GraphController(
        window, window.kinematics_controller.get_worker())
    window.graphsBox.layout().addWidget(window.graph_controller.get_widget())
    window.kinematics_controller.set_graph_controller(window.graph_controller)


def center_window(window):
    """Centra la ventana en la pantalla al iniciar la aplicación.

    Args:
        window: MainWindow propietaria.
    """
    screen = QApplication.primaryScreen()
    screen_geometry = screen.geometry()
    x = (screen_geometry.width() - window.width()) // 2
    y = (screen_geometry.height() - window.height()) // 2
    window.move(x, y)
