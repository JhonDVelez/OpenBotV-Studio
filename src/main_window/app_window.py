"""
Módulo principal de la interfaz gráfica de usuario.

Define la clase MainWindow, la ventana principal frameless de la
aplicación. La construcción de menús, layout y toolbar vive en módulos
de funciones (menu_builder, layout_builder, action_handlers, title_bar);
MainWindow conserva el estado y orquesta el ciclo de vida de los
sub-sistemas.

Conexiones:
    - Hereda de `FramelessMainWindow` para una estética moderna sin bordes.
    - Utiliza `ThemeManager` para la gestión de estilos claro/oscuro.
    - Se comunica con controladores de cámara, simulación, cinemática y sliders.
"""

from qframelesswindow import FramelessMainWindow
from PyQt6.QtWidgets import QMessageBox, QVBoxLayout, QWidget, QLabel, QApplication, QMainWindow
from PyQt6.QtCore import QCoreApplication, Qt

from src.main_window.action_handlers import wire_main_actions
from src.main_window import layout_builder, menu_builder
from src.main_window.title_bar import AppTitleBar
from src.services.devices import CameraDevices
from src.services.devices.device_monitor import get_device_monitor
from src.services.data.signals import bus
from src.services.styling import ThemeManager
from src.services.data import config_manager, DataController
from src.services.ui.notification_manager import notification_manager
from src.services.data.enums.types import NotificationType


class MainWindow(FramelessMainWindow):
    """
    Ventana principal de la aplicación de control del brazo robótico.

    La construcción de la interfaz (menú, docks, toolbar, barra de
    estado) está delegada a los módulos de funciones builders; esta
    clase guarda el estado y los controladores de la aplicación.
    """

    def __init__(self, quick3d, robot_id):
        """
        Inicializa la ventana principal y configura todos los sub-sistemas.

        Args:
            quick3d (PreloadedContainer): Datos y recursos pre-cargados de QML/Quick3D.
            robot_id (str): Identificador único para el robot (URDF/Config).
        """
        super().__init__()
        # Inicialización centralizada de configuración (antes de cualquier controller)
        self.config_manager = bus
        config_manager.init_config()
        for filename in config_manager.DEFAULTS.keys():
            data = config_manager.load(filename)
            bus.cfg_set_all_config(filename, data)

        # Orquestador Central de Datos e Inter-Controladores
        self.data_controller = DataController()

        self.preloaded_data = quick3d
        self.robot_id = robot_id
        self.simulation_controller = None
        self.stopped = True
        self.camera_paused = False
        self.hab_simulation = True
        self.theme_manager = ThemeManager(self)
        self.theme_signal_manager = bus
        bus.theme_changed.connect(
            self._on_external_theme_changed)
        self.com = None
        self.com_connected_label = QLabel("Micro no conectado")
        self.camera_connected_label = QLabel("Cámara no conectada")
        self.connected_to_robot = False
        self.last_cameras = None
        self.last_com = None
        self.last_camera_name = None

        # Crear contenedor principal para manejar barra de titulo personalizada y contenido
        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)

        # Barra de titulo personalizada
        menu_builder.build_main_menu(self)
        self.title_bar = AppTitleBar(self)
        self.title_bar.setObjectName("title_bar")
        self.setTitleBar(self.title_bar)
        container_layout.addWidget(self.title_bar)

        self.inner_window = QMainWindow()
        self.inner_window.setWindowFlags(Qt.WindowType.Widget)
        self.inner_window.setObjectName("inner_window")
        container_layout.addWidget(self.inner_window)

        # setup_ui gestiona los docks directamente en inner_window
        layout_builder.setup_ui(self)

        # Inicialización de servicios y controladores (módulos builders)
        menu_builder.build_main_status_bar(self)
        layout_builder.init_camera(self)
        layout_builder.init_controls(self)

        layout_builder.init_tool_bar(self)
        layout_builder.init_simulation(self)
        layout_builder.init_graphics(self)

        # Link kinematics to P&P
        self.pick_and_place_controller.set_kinematics_controller(self.kinematics_controller, self.graph_controller)

        wire_main_actions(self)

        # Añadir toolbar de acciones
        self.inner_window.addToolBar(
            Qt.ToolBarArea.LeftToolBarArea,
            self.actions_toolbar
        )

        # Cargar configuración de visibilidad de paneles desde archivo
        settings: dict = bus.cfg_get_param(
            "settings.json", default={})
        content = settings.get("content", {})
        mode = settings.get('mode', {})
        mapping_content = {
            "camera": (self.cameraDock, self.camera_action, True),
            "model": (self.modelDock, self.model_action, True),
            "graphs": (self.graphsDock, self.graphs_action, True),
            "controls": (self.controlsDock, self.controls_action, not mode.get("pick_place", False)),
        }

        for key, (dock, action, enable) in mapping_content.items():
            if key in content:
                state = bool(content[key])
                dock.toggleView(state)
                action.setChecked(state)
                action.setEnabled(bool(enable))

            # Sincronizar dock -> action (por si se cierra desde el botón 'X' del dock)
            dock.visibilityChanged.connect(action.setChecked)

        # Cargar configuración de búsqueda visual
        camera = settings.get("camera", {})
        search_manager = bus
        for key, widget in camera.items():
            state = bool(camera[key])
            if key == "charuco":
                bus.search_set_charuco(state)
            elif key == "circle":
                bus.search_set_circle(state)

        self.hab_simulation = settings.get(
            "simulation", {}).get("activated", True)

        # Siempre iniciar en modo Sliders al arrancar la aplicación
        self.sliders_controller.get_widget().show()
        self.kinematics_controller.get_widget().hide()

        self.sliders_action.setChecked(True)
        self.kinematics_action.setChecked(False)

        if self.graph_controller:
            self.graph_controller.set_graph_mode(True)

        # Conectar señal al cambio de tema del sistema
        QApplication.instance().styleHints().colorSchemeChanged.connect(
            self.theme_manager.update_theme)
        self.theme_manager.load_current_theme()

        # Instalar monitor de dispositivos (Cámaras y Puertos COM)
        self.camera_devices = CameraDevices()
        self._device_monitor = get_device_monitor(
            lambda: menu_builder.refresh_com_ports(self),
            self.camera_devices.get_cameras)
        self._device_monitor.install_filter(QCoreApplication.instance())

        menu_builder.refresh_com_ports(self)
        self.camera_devices.get_cameras()
        self.setCentralWidget(container)

        # Restaurar geometría desde config
        self.setMinimumSize(800, 600)
        settings = config_manager.load("settings.json")
        win_cfg = settings.get("window", {})
        if win_cfg.get("maximized"):
            self.showMaximized()
        else:
            w = win_cfg.get("width", 1280)
            h = win_cfg.get("height", 720)
            self.resize(w, h)
            layout_builder.center_window(self)

        notification_manager.set_main_window(self)

    def _on_external_theme_changed(self, is_dark: bool):
        """
        Actualiza el estilo cuando se detecta un cambio de tema externo.

        Args:
            is_dark (bool): True si el nuevo tema es oscuro.
        """
        self.theme_manager.apply_theme_from_signal(is_dark)

    def closeEvent(self, event):
        """
    Gestiona el cierre de la aplicación y la liberación de recursos.

    Muestra un diálogo de confirmación y asegura que los hilos de cámara,
    simulación y ventanas secundarias se detengan correctamente.

        Args:
            event (QCloseEvent): Evento de cierre de Qt.
        """
        msg = QMessageBox(self)
        msg.setWindowTitle("Salir")
        msg.setText("¿Seguro que quieres cerrar la aplicación?")
        msg.setIcon(QMessageBox.Icon.Question)

        msg.setWindowFlags(
            msg.windowFlags() | Qt.WindowType.WindowStaysOnTopHint
        )

        si_btn = msg.addButton("Sí", QMessageBox.ButtonRole.YesRole)
        no_btn = msg.addButton("No", QMessageBox.ButtonRole.NoRole)
        msg.setDefaultButton(no_btn)

        msg.exec()

        if msg.clickedButton() == si_btn:
            # Guardar geometría de ventana
            is_max = self.isMaximized()
            if not is_max:
                config_manager.set_value(
                    "settings.json", ["window", "width"], self.width())
                config_manager.set_value(
                    "settings.json", ["window", "height"], self.height())
            config_manager.set_value(
                "settings.json", ["window", "maximized"], is_max)

            # Detención segura de procesos
            if hasattr(self, "calibration_window"):
                if hasattr(self.calibration_window, "calibration_interface"):
                    self.calibration_window.calibration_interface.stop_video()
                self.calibration_window.close()
            if hasattr(self, "color_window"):
                if hasattr(self.color_window, "color_interface"):
                    self.color_window.color_interface.close()
                self.color_window.close()
            if hasattr(self, "camera_controller"):
                self.camera_controller.stop_video()
            if hasattr(self, 'graph_controller'):
                self.graph_controller.cleanup()
            event.accept()
        else:
            event.ignore()
