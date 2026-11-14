"""Introspeccion compartida para los snapshots golden (contratos de comportamiento).

El objetivo: capturar la superficie publica exacta de las senales y enums y
re-validarla tras cada fase del refactor. El golden
(`golden_snapshots.json`) fue capturado ANTES del refactor (Fase 0) y NO se
regenera al migrar a la nueva API.

Desde la Fase 1 los managers por dominio fueron sustituidos por un bus unico
(`SignalBus` con la instancia `bus`). Este adaptador mapea cada clase antigua
del golden a sus senales en el bus; si una senal desaparece, cambia de firma
o se renombra sin intencion, el test falla.

Regenerar goldens (solo cuando el cambio es INTENCIONAL y revisado):
    python tests/golden/generate_golden.py
"""

from pathlib import Path

GOLDEN_DIR = Path(__file__).resolve().parent

# Senales built-in de QObject: presentes en toda instancia, no forman parte
# de los contratos de dominio, se filtran para mantener el golden estable.
QOBJECT_BUILTIN_SIGNALS = {"destroyed", "objectNameChanged"}

# Senales de la antigua clase base _SignalManager (la heredaban 4 dominios,
# cada uno con INSTANCIAS independientes; en el bus cada dominio tiene su
# copia prefijada).
_BASE_SIGNALS = [
    "get_data_signal",
    "sensor_position_signal",
    "model_position_signal",
    "update_robot_signal",
    "update_graph_signal",
    "update_pybullet_signal",
    "update_target_signal",
    "change_mode_signal",
]

# Mapa de adaptacion clase antigua -> {senal antigua -> atributo del bus}.
BUS_SIGNAL_MAP = {
    "_SignalManager": {s: f"sld_{s}" for s in _BASE_SIGNALS},
    "SlidersSignalManager": {s: f"sld_{s}" for s in _BASE_SIGNALS},
    "SimulationSignalManager": (
        {s: f"sim_{s}" for s in _BASE_SIGNALS} | {
            "sphere_pos_from_camera": "sim_sphere_pos_from_camera",
            "sphere_pos_from_pybullet": "sim_sphere_pos_from_pybullet",
            "clear_spheres": "sim_clear_spheres",
            "release_sphere": "sim_release_sphere",
            "reattach_sphere": "sim_reattach_sphere",
            "sphere_radius_changed": "sim_sphere_radius_changed",
            "start_simulation": "sim_start_simulation",
            "pause_simulation": "sim_pause_simulation",
            "resume_simulation": "sim_resume_simulation",
            "stop_simulation": "sim_stop_simulation",
            "update_robot_from_kinematics": "sim_update_robot_from_kinematics",
            "update_robot_from_sliders": "sim_update_robot_from_sliders",
            "update_robot_from_simulation": "sim_update_robot_from_simulation",
            "start_request": "sim_start_request",
            "pause_request": "sim_pause_request",
            "stop_request": "sim_stop_request",
        }
    ),
    "PhysicalSignalManager": (
        {s: f"phy_{s}" for s in _BASE_SIGNALS} | {
            "send_to_robot": "phy_send_to_robot",
            "data_received": "phy_data_received",
            "start_service": "phy_start_service",
            "stop_service": "phy_stop_service",
            "pause_service": "phy_pause_service",
            "resume_service": "phy_resume_service",
            "start_request": "phy_start_request",
            "stop_request": "phy_stop_request",
        }
    ),
    "KinematicsSignalManager": (
        {s: f"kin_{s}" for s in _BASE_SIGNALS} | {
            "inverse_kinematics_requested": "kin_inverse_kinematics_requested",
            "inverse_kinematics_ready": "kin_inverse_kinematics_ready",
        }
    ),
    "CameraSignalManager": {
        "available_cameras": "cam_available_cameras",
        "charuco_done": "cam_charuco_done",
        "circles_done": "cam_circles_done",
        "pose_done": "cam_pose_done",
        "spheres_detected_2d": "cam_spheres_detected_2d",
        "poses_from_camera": "cam_poses_from_camera",
        "clear_spheres_request": "cam_clear_spheres_request",
    },
    "ConfigSignalManager": {
        "config_updated": "cfg_config_updated",
        "change_requested": "cfg_change_requested",
    },
    "PickPlaceSignalManager": {
        "poses_from_camera": "pp_poses_from_camera",
        "spheres_detected_2d": "pp_spheres_detected_2d",
        "sphere_selected": "pp_sphere_selected",
        "place_requested": "pp_place_requested",
        "pick_requested": "pp_pick_requested",
        "inverse_kinematics_requested": "pp_inverse_kinematics_requested",
        "inverse_kinematics_ready": "pp_inverse_kinematics_ready",
        "target_reached": "pp_target_reached",
        "state_changed": "pp_state_changed",
        "change_mode_signal": "pp_change_mode_signal",
        "pick_place_running_changed": "pp_pick_place_running_changed",
        "release_sphere_request": "pp_release_sphere_request",
        "reattach_sphere_request": "pp_reattach_sphere_request",
        "clear_spheres_request": "pp_clear_spheres_request",
        "search_circle_request": "pp_search_circle_request",
    },
    "SearchSignalManager": {
        "charuco_search_changed": "search_charuco_changed",
        "circle_search_changed": "search_circle_changed",
    },
    "ThemeSignalManager": {
        "theme_changed": "theme_changed",
    },
}


def enum_classes():
    """Importa los enums con contrato estable (Fase 3)."""
    from src.services.data.enums.types import Domains, Modes, NotificationType, Units
    from src.features.pick_and_place.pick_place_states import PickPlaceState

    return [Modes, Units, Domains, NotificationType, PickPlaceState]


def bus_signal_surface():
    """Snapshot de TODAS las senales del bus: {nombre: [tipos_cpp]}."""
    from src.services.data.signals import bus
    from PyQt6.QtCore import QMetaMethod

    obj = bus
    meta = obj.metaObject()
    out = {}
    for i in range(meta.methodCount()):
        m = meta.method(i)
        if m.methodType() != QMetaMethod.MethodType.Signal:
            continue
        name = bytes(m.name()).decode()
        if name in QOBJECT_BUILTIN_SIGNALS:
            continue
        out[name] = [bytes(t).decode() for t in m.parameterTypes()]
    return out


def manager_signal_contracts():
    """Contratos de senales en formato golden, leidos del bus nuevo.

    Devuelve {clase_antigua: {senal_antigua: [tipos_cpp]}} resolviendo cada
    senal a traves del mapa de adaptacion. Lanza AssertionError con un
    mensaje claro si el bus no expone alguna senal mapeada.
    """
    surface = bus_signal_surface()
    out = {}
    for cls_name, mapping in BUS_SIGNAL_MAP.items():
        contract = {}
        for old_name, bus_attr in mapping.items():
            if bus_attr not in surface:
                raise AssertionError(
                    f"El bus no expone '{bus_attr}' (mapeo de {cls_name}.{old_name})")
            contract[old_name] = surface[bus_attr]
        out[cls_name] = contract
    return out


def collect_enum(cls):
    """Snapshot de miembros de un Enum: {miembro: valor}."""
    return {m.name: m.value for m in cls}


def snapshot_all():
    """Snapshot completo de managers y enums como diccionario serializable."""
    managers = manager_signal_contracts()
    enums = {c.__name__: collect_enum(c) for c in enum_classes()}
    return {"managers": managers, "enums": enums}


def load_golden():
    """Carga el golden commiteado como diccionario."""
    import json

    return json.loads((GOLDEN_DIR / "golden_snapshots.json").read_text(encoding="utf-8"))
