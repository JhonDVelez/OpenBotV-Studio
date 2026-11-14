"""
Componentes compartidos de la decoración de ventanas (window chrome).

Funciones constructoras (builder) para los elementos visuales comunes a
todas las ventanas de la aplicación: barra de menús, acción de tema y
barra de estado. Sustituyen a los antiguos mixins duplicados
(CalibrationMenuMixin / ColorMenuMixin / MainMenuMixin) por funciones
planas que operan sobre la ventana recibida.
"""

import os

from PyQt6.QtGui import QAction, QKeySequence, QIcon
from PyQt6.QtWidgets import QMenuBar, QStatusBar

__all__ = [
    "build_menu_bar",
    "build_theme_action",
    "build_status_bar",
]


def build_menu_bar(window) -> QMenuBar:
    """Crea y devuelve la barra de menús estándar de la aplicación.

    Altura fija de 32 px y sin márgenes, pensada para integrarse en la
    barra de título personalizada.

    Args:
        window: Ventana propietaria (se usa como parent del menú).

    Returns:
        QMenuBar: Barra de menús configurada.
    """
    menu_bar = QMenuBar(window)
    menu_bar.setFixedHeight(32)
    menu_bar.setContentsMargins(0, 0, 0, 0)
    menu_bar.adjustSize()
    return menu_bar


def build_theme_action(window) -> QAction:
    """Crea la acción de cambio de tema y la registra en la ventana.

    Carga los iconos sol/luna (dark/light) y los expone como atributos
    ``sun_icon`` y ``moon_icon`` de la ventana, igual que hacían los
    antiguos mixins, para que la barra de título y los handlers de tema
    puedan usarlos.

    Args:
        window: Ventana propietaria de la acción.

    Returns:
        QAction: Acción de cambio de tema (atajo Ctrl+T).
    """
    window.sun_icon = QIcon(os.path.join("icons:sun.png"))
    window.moon_icon = QIcon(os.path.join("icons:moon.png"))

    theme_action = QAction("", window)
    theme_action.setShortcut(QKeySequence("Ctrl+t"))
    theme_action.setStatusTip("Cambiar tema")

    window.theme_action = theme_action
    return theme_action


def build_status_bar(window, *permanent_widgets) -> QStatusBar:
    """Crea e integra una barra de estado en la ventana.

    Args:
        window: Ventana en la que se instala la barra de estado.
        *permanent_widgets: Widgets fijos (lado derecho) a añadir.

    Returns:
        QStatusBar: Barra de estado creada.
    """
    status_bar = QStatusBar(window)
    window.setStatusBar(status_bar)
    for widget in permanent_widgets:
        status_bar.addPermanentWidget(widget)
    return status_bar
