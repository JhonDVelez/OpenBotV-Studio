"""
Paquete encargado de la ventana principal de la aplicación.

Proporciona la clase MainWindow (ventana principal frameless). La
construcción de menús, layout, toolbar y acciones vive en módulos de
funciones planas (menu_builder, layout_builder, action_handlers,
title_bar) dentro de este mismo paquete.
"""

from src.main_window.app_window import MainWindow

__all__ = [
    "MainWindow",
]
