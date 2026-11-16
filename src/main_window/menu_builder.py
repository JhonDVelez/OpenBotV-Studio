"""
Construcción del menú principal de la ventana principal (MainWindow).

Sustituye al antiguo MainMenuMixin por funciones planas que operan sobre
la ventana recibida. Crea las acciones config-driven, los submenús de
cámara, modo, simulación y puertos COM, y la barra de estado con
indicadores de conexión.
"""

import os
from collections import Counter

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QActionGroup, QKeySequence, QPixmap
from PyQt6.QtWidgets import (
    QFrame, QInputDialog, QLabel, QMenuBar, QSizePolicy)
from serial.tools import list_ports

from src.services.data.signals import bus
from src.services.data.timers import frame_counter
from src.services.ui import window_chrome


def build_main_menu(window):
    """Construye el menú principal y sus submenús.

    Crea la barra de menú con las secciones de Cámara, Modo,
    Simulación y Robot, integrando el logo de la aplicación.

    Args:
        window: MainWindow propietaria. Los QAction creados se registran
            como atributos de la ventana (sliders_action, mode_menu, ...).
    """
    build_main_actions(window)

    window.menu_bar = QMenuBar()
    window.menu_bar.setFixedHeight(32)
    window.menu_bar.setContentsMargins(0, 0, 0, 0)
    window.menu_bar.adjustSize()

    window.logo_label = QLabel()
    window.laser_w = QPixmap("img:laser_w.png").scaledToHeight(
        20, Qt.TransformationMode.SmoothTransformation)
    window.laser_b = QPixmap("img:laser_b.png").scaledToHeight(
        20, Qt.TransformationMode.SmoothTransformation)
    window.logo_label.setPixmap(window.laser_w)
    window.logo_label.setContentsMargins(8, 0, 5, 0)

    window.menu_bar.setCornerWidget(
        window.logo_label, Qt.Corner.TopLeftCorner)

    window.camera_menu = window.menu_bar.addMenu("&Cámara")
    window.camera_menu.addAction(window.camera_calibration_action)

    window.sphere_menu = window.menu_bar.addMenu("&Esfera")
    window.sphere_menu.addAction(window.color_calibration_action)
    window.sphere_size_submenu = window.sphere_menu.addMenu("&Tamaño")
    window.sphere_size_group = QActionGroup(window)
    window.sphere_size_group.setExclusive(True)

    presets_size = [20.0, 30.0]
    current_size = bus.cfg_get_param(
        "camera.json", "sphere_radius", default=20.0)

    size_found = False
    for size in presets_size:
        action = window.sphere_size_submenu.addAction(f"{int(size)}mm")
        action.setCheckable(True)
        window.sphere_size_group.addAction(action)
        if abs(size - current_size) < 0.1:
            action.setChecked(True)
            size_found = True
        action.triggered.connect(
            lambda checked, s=size: handle_sphere_size(window, s))

    window.custom_size_action = window.sphere_size_submenu.addAction(
        "Personalizado...")
    window.custom_size_action.setCheckable(True)
    window.sphere_size_group.addAction(window.custom_size_action)
    if not size_found:
        window.custom_size_action.setChecked(True)
        window.custom_size_action.setText(
            f"Personalizado ({current_size}mm)")
    window.custom_size_action.triggered.connect(
        lambda: open_custom_sphere_size(window))

    window.camera_interval_submenu = window.camera_menu.addMenu(
        "&Intervalo")
    window.camera_interval_group = QActionGroup(window)
    window.camera_interval_group.setExclusive(True)
    presets_interval = [1, 2, 4, 10]

    pre_interval = bus.cfg_get_param(
        "settings.json", "camera", "view", "interval", default=4)

    for preset in presets_interval:
        action = window.camera_interval_submenu.addAction(f"{preset}")
        action.setCheckable(True)
        window.camera_interval_group.addAction(action)
        if preset == pre_interval:
            action.setChecked(True)
        action.triggered.connect(
            lambda checked, i=preset: apply_camera_interval(i))
    window.camera_interval_submenu.setEnabled(False)

    window.mode_menu = window.menu_bar.addMenu("&Modo")
    window.mode_group = QActionGroup(window.mode_menu)
    window.mode_menu.addAction(window.sliders_action)
    window.mode_menu.addAction(window.kinematics_action)
    window.mode_menu.addAction(window.pick_place_action)
    window.mode_group.addAction(window.sliders_action)
    window.mode_group.addAction(window.kinematics_action)
    window.mode_group.addAction(window.pick_place_action)
    window.mode_group.setExclusive(True)

    window.simulation_menu = window.menu_bar.addMenu("&Simulación")
    window.simulation_menu.addAction(window.simulation_action)
    window.simulation_menu.addSeparator()
    window.simulation_menu.addAction(window.shadows_action)
    window.simulation_menu.addAction(window.aa_action)
    window.simulation_menu.addAction(window.grid_action)
    window.simulation_menu.addAction(window.axes_action)
    window.simulation_menu.addAction(window.labels_action)

    window.robot_menu = window.menu_bar.addMenu("&Robot")
    window.com_submenu = window.robot_menu.addMenu("&Puerto")
    window.com_group = QActionGroup(window)
    window.com_group.setExclusive(True)
    refresh_com_ports(window)
    window.robot_menu.addAction(window.connect_action)

    window.menu_bar.setSizePolicy(
        QSizePolicy(QSizePolicy.Policy.Preferred,
                    QSizePolicy.Policy.Preferred)
    )


def build_main_actions(window):
    """Define las acciones del menú con atajos, textos de estado e iconos.

    Lee la configuración guardada en settings.json para inicializar
    el estado de las acciones checkables.

    Args:
        window: MainWindow propietaria de las acciones.
    """
    settings = bus.cfg_get_param("settings.json", default={})

    # nombre_menu, nombre_opcion, nombre_objeto, nombre_interfaz_1, nombre_interfaz_2, shortcut,
    # descripción para barra de estado y si está o no el dato almacenado en config_manager
    mapping_mode = {
        "mode": {
            "sliders": ("sliders_action", "Sliders",
                        "Sliders", "Ctrl+a",
                        "Mostrar/Ocultar controles con sliders", True),
            "kinematics": ("kinematics_action", "Cinemática",
                           "Cinemática", "Ctrl+s",
                           "Mostrar/Ocultar controles de cinemática", True),
            "pick_place": ("pick_place_action", "Pick and Place",
                           "Pick and Place", "Ctrl+d",
                           "Mostrar/Ocultar controles de pick and place", True),
        }
    }

    mapping_camera = {
        "camera": {
            "calibrate": ("camera_calibration_action", "Calibrar Cámara", "Calibrar Cámara",
                          "", "Abrir ventana de calibración de cámara", False),
            "color_calibrate": ("color_calibration_action", "Calibrar Color", "Calibrar Color",
                                "", "Abrir ventana de calibración de colores", False),
        }
    }

    mapping_simulation = {
        "simulation": {
            "activated": ("simulation_action", "Activar Simulación",
                          "Activar Simulación", "Ctrl+y",
                          "Activar/Desactivar simulación", True),
            "shadows": ("shadows_action", "Sombras",
                        "Sombras", "Ctrl+h",
                        "Activar/Desactivar sombras en la escena", True),
            "grid": ("grid_action", "Malla Infinita",
                     "Malla Infinita", "Ctrl+g",
                     "Mostrar/Ocultar malla infinita", True),
            "axes": ("axes_action", "Ejes de Coordenadas",
                     "Ejes de Coordenadas", "Ctrl+j",
                     "Mostrar/Ocultar ejes de coordenadas", True),
            "labels": ("labels_action", "Etiquetas de Juntas",
                       "Etiquetas de Juntas", "Ctrl+k",
                       "Mostrar/Ocultar ángulos de las articulaciones", True),
            "aa": ("aa_action", "Antialiasing (MSAA)",
                   "Antialiasing (MSAA)", "Ctrl+l",
                   "Activar/Desactivar suavizado de bordes", True),
        }
    }

    mapping_all = (mapping_mode, mapping_camera, mapping_simulation)

    for mapping in mapping_all:
        for main_key, creation_data in mapping.items():
            saved_config = settings.get(main_key)
            for key, (attr_name, label_show, label_hide, shortcut, status, is_checkable) in creation_data.items():
                action = None
                if key in saved_config:
                    if saved_config.get(key):
                        action = QAction(label_hide, window)
                    else:
                        action = QAction(label_show, window)
                    action.setChecked(saved_config.get(key))
                else:
                    action = QAction(label_hide, window)
                if action is not None:
                    action.setCheckable(is_checkable)
                    action.setShortcut(QKeySequence(shortcut))
                    action.setStatusTip(status)
                    setattr(window, attr_name, action)

    window_chrome.build_theme_action(window)

    window.connect_action = QAction("Conectar", window)
    window.connect_action.setEnabled(False)

    window.theme_signal_manager = bus
    bus.theme_changed.connect(
        lambda dark_t: update_theme_icon(window, dark_t))


def refresh_com_ports(window):
    """Escanea los puertos serie disponibles y actualiza el submenú.

    Expone los puertos COM encontrados como acciones seleccionables
    en el submenú de Robot.

    Args:
        window: MainWindow propietaria del submenú de puertos.
    """
    available_ports = list_ports.comports()
    available_com = [
        port for port in available_ports if port.description != 'n/a']
    if window.last_com is not None and Counter(window.last_com) == Counter(available_com):
        return

    window.last_com = available_com

    window.com_submenu.clear()
    for action in list(window.com_group.actions()):
        window.com_group.removeAction(action)

    if available_com:
        if window.stopped:
            window.com_submenu.setEnabled(False)
        else:
            window.com_submenu.setEnabled(True)
        for port in available_com:
            com_action = window.com_submenu.addAction(port.description)
            com_action.setCheckable(True)
            com_action.setData(port.device)
            com_action.setStatusTip(f"Conectar al puerto {port.device}")
            window.com_group.addAction(com_action)
            com_action.triggered.connect(
                lambda checked: handle_com_selected(window, checked))

            if window.com == port.device:
                com_action.setChecked(True)
    else:
        window.connect_action.setEnabled(False)
        window.com_submenu.setEnabled(False)
        window.com = None
        stop_robot_threads(window)
        window.com_connected_label.setText("Micro no conectado")


def handle_com_selected(window, checked):
    """Maneja la selección de un puerto COM en el submenú.

    Si se selecciona un nuevo puerto, detiene la conexión anterior
    y habilita el botón de conectar.

    Args:
        window: MainWindow propietaria.
        checked (bool): Indica si la acción está seleccionada.
    """
    action = window.sender()
    if not window.com:
        stop_robot_threads(window)
    if action and checked:
        window.com = action.data()
        if (not getattr(window, "robot_service", None) and
            not getattr(window, "robot_controller", None) and
                not window.stopped):
            window.connect_action.setEnabled(True)


def apply_camera_interval(interval):
    """Aplica el intervalo de captura de cámara seleccionado.

    Args:
        interval (int): Nuevo intervalo en ticks de frame.
    """
    frame_counter.set_interval(interval)


def handle_sphere_size(window, size):
    """Actualiza el radio de la esfera en la configuración.

    Args:
        window: MainWindow propietaria.
        size (float): Nuevo radio en mm.
    """
    bus.cfg_request_change(
        "camera.json", ["sphere_radius"], float(size))
    window.custom_size_action.setText("Personalizado...")


def open_custom_sphere_size(window):
    """Abre un diálogo para ingresar un tamaño de esfera personalizado.

    Args:
        window: MainWindow propietaria.
    """
    current = bus.cfg_get_param(
        "camera.json", "sphere_radius", default=20.0)

    val, ok = QInputDialog.getDouble(
        window, "Tamaño de Esfera", "Radio en mm:",
        current, 1.0, 500.0, 1
    )

    if ok:
        handle_sphere_size(window, val)
        window.custom_size_action.setChecked(True)
        window.custom_size_action.setText(f"Personalizado ({val}mm)")
    else:
        # Re-check the previous action if cancelled
        # This is tricky because we don't know which one was checked.
        # But the QActionGroup should handle the exclusivity.
        # If we cancelled, we should probably restore the check on the current size.
        pass


def stop_robot_threads(window):
    """Detiene y libera los hilos del robot de forma segura.

    Args:
        window: MainWindow propietaria de los controladores de robot.
    """
    if getattr(window, "robot_controller", None):
        try:
            if hasattr(window.robot_controller, 'stop'):
                window.robot_controller.stop()
            window.robot_controller.deleteLater()
        except RuntimeError:
            pass
        finally:
            window.robot_controller = None

    if getattr(window, "robot_service", None):
        try:
            window.robot_service.stop_service()
            window.robot_service.deleteLater()
        except RuntimeError:
            pass
        finally:
            window.robot_service = None


def build_main_status_bar(window):
    """Crea la barra de estado con indicadores de conexión de cámara y COM.

    Args:
        window: MainWindow propietaria. Debe tener ya creados
            ``camera_connected_label`` y ``com_connected_label``.
    """
    separator = QFrame()
    separator.setFrameShape(QFrame.Shape.VLine)
    window_chrome.build_status_bar(
        window,
        window.camera_connected_label,
        separator,
        window.com_connected_label,
    )


def update_theme_icon(window, dark_t: bool):
    """Actualiza el icono del botón de tema según el modo actual.

    Args:
        window: MainWindow propietaria del theme_action.
        dark_t (bool): True si el tema es oscuro.
    """
    if dark_t:
        window.theme_action.setIcon(window.sun_icon)
    else:
        window.theme_action.setIcon(window.moon_icon)
