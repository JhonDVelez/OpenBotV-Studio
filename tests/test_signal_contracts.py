"""Contratos de comportamiento del sistema de senales (Fase 1 del refactor).

Estos tests capturan el comportamiento ANTES de consolidar los 12 managers en
un SignalBus unico. Tras el refactor, `manager_classes()` se adapta a la nueva
API y estos mismos tests deben seguir en verde SIN regenerar el golden.
"""

import threading

import pytest
from PyQt6.QtCore import QThread

from tests.golden.registry import load_golden, manager_classes, collect_signals


# ---------------------------------------------------------------------------
# Golden snapshot: la superficie publica no cambia sin intencion
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_signal_surface_matches_golden():
    golden = load_golden()["managers"]
    current = {c.__name__: collect_signals(c) for c in manager_classes()}

    missing = set(golden) - set(current)
    added = set(current) - set(golden)
    assert not missing, f"Clases de senales desaparecidas: {sorted(missing)}"
    assert not added, f"Clases de senales nuevas: {sorted(added)}"

    for cls_name in golden:
        g_signals, c_signals = golden[cls_name], current[cls_name]
        missing_sig = set(g_signals) - set(c_signals)
        added_sig = set(c_signals) - set(g_signals)
        assert not missing_sig, f"{cls_name}: senales desaparecidas {sorted(missing_sig)}"
        assert not added_sig, f"{cls_name}: senales nuevas {sorted(added_sig)}"
        for sig_name, g_types in g_signals.items():
            assert c_signals[sig_name] == g_types, (
                f"{cls_name}.{sig_name}: firma cambio "
                f"{g_types} -> {c_signals[sig_name]}"
            )


# ---------------------------------------------------------------------------
# Comportamiento en vivo: emitir/recibir sigue funcionando igual
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_direct_connection_round_trip():
    """Una senal tipada emite y entrega el payload intacto (conexion directa)."""
    from src.services.data.signals import CameraSignalManager

    manager = CameraSignalManager()
    received = []
    manager.spheres_detected_2d.connect(lambda data: received.append(data))

    payload = {"red": {"position": [1.0, 2.0, 3.0]}}
    manager.spheres_detected_2d.emit(payload)

    assert received == [payload]


def test_cross_thread_queued_delivery(qapp):
    """Emision desde un QThread se entrega en el hilo principal (queued).

    Este es el contrato central del sistema de senales: workers en hilos
    comunican resultados a la UI sin acoplamiento. Debe seguir tras el bus.
    """
    from src.services.data.signals import CameraSignalManager

    manager = CameraSignalManager()
    received = []
    manager.spheres_detected_2d.connect(lambda data: received.append(data))

    payload = {"blue": {"position": [4.0, 5.0, 6.0]}}

    class Worker(QThread):
        def run(self):
            manager.spheres_detected_2d.emit(payload)

    worker = Worker()
    worker.start()
    worker.wait(10_000)

    deadline = 5.0
    while not received and deadline > 0:
        qapp.processEvents()
        deadline -= 0.05
        threading.Event().wait(0.05)

    assert received == [payload]


# ---------------------------------------------------------------------------
# Singletons actuales: identidad por clase (contrato que la Fase 1 elimina
# a proposito, pero mientras exista debe comportarse asi)
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_singleton_identity():
    """get_instance() devuelve siempre el mismo objeto dentro de un hilo."""
    from src.services.data.signals import CameraSignalManager, ConfigSignalManager

    assert CameraSignalManager.get_instance() is CameraSignalManager.get_instance()
    assert ConfigSignalManager.get_instance() is ConfigSignalManager.get_instance()
    # Instancias independientes entre dominios
    assert CameraSignalManager.get_instance() is not ConfigSignalManager.get_instance()


@pytest.mark.usefixtures("qapp")
def test_config_signal_manager_is_pure_memory():
    """set_all_config/get_param/update_param operan solo en el cache en memoria.

    Contrato clave: la configuracion leida/escrita por el manager no toca
    disco (la persistencia la pide via signal change_requested).
    """
    from src.services.data.signals import ConfigSignalManager

    manager = ConfigSignalManager()
    manager.set_all_config("settings.json", {"camera": {"view": {"charuco": False}}})

    assert manager.get_param("settings.json", "camera", "view", "charuco") is False
    assert manager.get_param("settings.json", "no", "existe", default="d") == "d"

    updated = []
    manager.config_updated.connect(lambda f, k, v: updated.append((f, k, v)))
    manager.update_param("settings.json", ["camera", "view", "charuco"], True)
    assert manager.get_param("settings.json", "camera", "view", "charuco") is True
    assert updated == [("settings.json", ["camera", "view", "charuco"], True)]

    requested = []
    manager.change_requested.connect(lambda f, k, v: requested.append((f, k, v)))
    manager.request_change("settings.json", ["camera", "view", "circle"], True)
    assert requested == [("settings.json", ["camera", "view", "circle"], True)]
