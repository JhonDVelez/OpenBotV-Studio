"""Contratos de comportamiento del sistema de señales.

Fase 0 capturó el comportamiento de los 12 managers por dominio. Desde la
Fase 1 todo vive en el `SignalBus` único; estos tests re-validan la misma
superficie contra el golden SIN regenerarlo (el adaptador está en
`tests/golden/registry.py`).
"""

import threading

import pytest
from PyQt6.QtCore import QThread

from tests.golden.registry import load_golden, manager_signal_contracts


# ---------------------------------------------------------------------------
# Golden snapshot: la superficie pública no cambia sin intención
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_signal_surface_matches_golden():
    golden = load_golden()["managers"]
    current = manager_signal_contracts()

    missing = set(golden) - set(current)
    added = set(current) - set(golden)
    assert not missing, f"Clases de señales desaparecidas: {sorted(missing)}"
    assert not added, f"Clases de señales nuevas: {sorted(added)}"

    for cls_name in golden:
        g_signals, c_signals = golden[cls_name], current[cls_name]
        missing_sig = set(g_signals) - set(c_signals)
        added_sig = set(c_signals) - set(g_signals)
        assert not missing_sig, f"{cls_name}: señales desaparecidas {sorted(missing_sig)}"
        assert not added_sig, f"{cls_name}: señales nuevas {sorted(added_sig)}"
        for sig_name, g_types in g_signals.items():
            assert c_signals[sig_name] == g_types, (
                f"{cls_name}.{sig_name}: firma cambió "
                f"{g_types} -> {c_signals[sig_name]}"
            )


@pytest.mark.usefixtures("qapp")
def test_bus_is_single_module_instance():
    """Contrato nuevo de la Fase 1: una única instancia `bus` en el proceso."""
    import src.services.data.signals as signals_pkg

    assert signals_pkg.bus is not None
    # Misma instancia a través de cualquier ruta de import
    from src.services.data.signals import bus as bus_direct
    from src.services.data.signals.bus import bus as bus_module
    assert signals_pkg.bus is bus_direct is bus_module


@pytest.mark.usefixtures("qapp")
def test_old_manager_classes_are_gone():
    """Los antiguos gestores por dominio ya no existen (Fase 1 los elimina)."""
    with pytest.raises(ImportError):
        import src.services.data.signals.camera  # noqa: F401
    with pytest.raises(ImportError):
        from src.services.data.signals import CameraSignalManager  # noqa: F401


# ---------------------------------------------------------------------------
# Comportamiento en vivo: emitir/recibir sigue funcionando igual
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_direct_connection_round_trip():
    """Una señal tipada emite y entrega el payload intacto (conexión directa)."""
    from src.services.data.signals import bus

    received = []
    bus.cam_spheres_detected_2d.connect(lambda data: received.append(data))

    payload = {"red": {"position": [1.0, 2.0, 3.0]}}
    bus.cam_spheres_detected_2d.emit(payload)

    assert received == [payload]


@pytest.mark.usefixtures("qapp")
def test_cross_thread_queued_delivery():
    """Emisión desde un QThread se entrega en el hilo principal (queued).

    Este es el contrato central del sistema de señales: workers en hilos
    comunican resultados a la UI sin acoplamiento. Debe seguir tras el bus.
    """
    from src.services.data.signals import bus

    received = []
    bus.cam_spheres_detected_2d.connect(lambda data: received.append(data))

    payload = {"blue": {"position": [4.0, 5.0, 6.0]}}

    class Worker(QThread):
        def run(self):
            bus.cam_spheres_detected_2d.emit(payload)

    worker = Worker()
    worker.start()
    worker.wait(10_000)

    deadline = 5.0
    from PyQt6.QtCore import QCoreApplication
    app = QCoreApplication.instance()
    while not received and deadline > 0:
        app.processEvents()
        deadline -= 0.05
        threading.Event().wait(0.05)

    assert received == [payload]


# ---------------------------------------------------------------------------
# Estado del bus: config, pick and place, búsqueda (contratos equivalentes
# a los antiguos singletons, ahora con métodos prefijados)
# ---------------------------------------------------------------------------

@pytest.mark.usefixtures("qapp")
def test_config_state_is_pure_memory():
    """cfg_set_all_config/cfg_get_param/cfg_update_param operan solo en cache.

    Contrato clave: la configuración leída/escrita por el bus no toca disco
    (la persistencia la pide via señal cfg_change_requested).
    """
    from src.services.data.signals import bus

    bus.cfg_set_all_config("settings.json", {"camera": {"view": {"charuco": False}}})

    assert bus.cfg_get_param("settings.json", "camera", "view", "charuco") is False
    assert bus.cfg_get_param("settings.json", "no", "existe", default="d") == "d"

    updated = []
    bus.cfg_config_updated.connect(lambda f, k, v: updated.append((f, k, v)))
    bus.cfg_update_param("settings.json", ["camera", "view", "charuco"], True)
    assert bus.cfg_get_param("settings.json", "camera", "view", "charuco") is True
    assert updated == [("settings.json", ["camera", "view", "charuco"], True)]

    requested = []
    bus.cfg_change_requested.connect(lambda f, k, v: requested.append((f, k, v)))
    bus.cfg_request_change("settings.json", ["camera", "view", "circle"], True)
    assert requested == [("settings.json", ["camera", "view", "circle"], True)]


@pytest.mark.usefixtures("qapp")
def test_pick_place_state_contract():
    """pp_set_state emite state_changed + change_mode_signal con el modo."""
    from src.services.data.enums import Modes
    from src.services.data.signals import bus

    states, modes = [], []
    bus.pp_state_changed.connect(lambda s: states.append(s))
    bus.pp_change_mode_signal.connect(lambda m: modes.append(m))

    bus.pp_set_state(True)
    bus.pp_set_state(False)

    assert states == [True, False]
    assert modes == [Modes.PICK_PLACE, Modes.SLIDERS]
    assert bus.pp_get_state() is False

    assert bus.pp_is_running() is False
    running = []
    bus.pp_pick_place_running_changed.connect(lambda r: running.append(r))
    bus.pp_set_running(True)
    assert bus.pp_is_running() is True
    assert running == [True]
