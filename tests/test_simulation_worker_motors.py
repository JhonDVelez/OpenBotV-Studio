"""
Tests de caracterización del cacheo de motores QML en SimulationWorker.

Los 6 motores del modelo 3D se resuelven UNA vez al construir el worker
(findChild) y cada aplicación posterior usa la caché, sin recorrer el
árbol QML por fotograma.
"""

import pytest
from PyQt6.QtCore import QObject

from src.features.simulation.simulation_worker import SimulationWorker

NOMBRES = [
    "arm1_link_1",
    "arm2_link_1",
    "arm3_link_1",
    "arm4_link_1",
    "clamp_arm_link_1",
    "clamp2_link_1",
]

# Eje de giro por motor (espejo de SimulationWorker.direction_rotation).
EJES = ["y", "z", "z", "y", "z", "x"]

ANGULOS = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]


class RootContador(QObject):
    """QObject raíz que cuenta las búsquedas findChild (que hoy se hacen por fotograma)."""

    def __init__(self):
        super().__init__()
        self.findchild_count = 0

    def findChild(self, cls, name):  # noqa: N802 (convención Qt)
        self.findchild_count += 1
        return super().findChild(cls, name)


def test_aplica_rotaciones_sin_findchild_por_frame(qapp):
    root = RootContador()
    hijos = []
    for nombre in NOMBRES:
        hijo = QObject(root)
        hijo.setObjectName(nombre)
        hijos.append(hijo)

    w = SimulationWorker(root, "robot")

    # 1ª aplicación: la única que puede buscar en el árbol QML.
    w.update_simulation(list(ANGULOS))
    w._process_pending_update()
    assert root.findchild_count == len(NOMBRES)

    # Rotaciones aplicadas, con el flip de los índices 4 y 5.
    esperados = [10.0, 20.0, 30.0, 40.0, -50.0, -60.0]
    for hijo, angulo, eje in zip(hijos, esperados, EJES):
        v = hijo.property("eulerRotation")
        assert v is not None, hijo.objectName()
        componentes = {"x": v.x(), "y": v.y(), "z": v.z()}
        for eje_comp, valor in componentes.items():
            esperado = angulo if eje_comp == eje else 0.0
            assert abs(valor - esperado) < 1e-6, (hijo.objectName(), eje_comp)

    # 2ª aplicación: ninguna búsqueda nueva en el árbol QML.
    w.update_simulation(list(ANGULOS))
    w._process_pending_update()
    assert root.findchild_count == len(NOMBRES)
