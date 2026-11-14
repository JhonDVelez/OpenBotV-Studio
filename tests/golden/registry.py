"""Introspeccion compartida para los snapshots golden (contratos de comportamiento).

El objetivo: ANTES de refactorizar, capturar la superficie publica exacta de
las clases que van a cambiar (senales de los managers y enums). Tras cada fase
del refactor, los mismos tests re-validan la nueva API contra estos snapshots,
de modo que cualquier cambio accidental de nombre o firma de senal falle.

Regenerar goldens (solo cuando el cambio es INTENCIONAL):
    python tests/golden/generate_golden.py
"""

from pathlib import Path

from PyQt6.QtCore import QMetaMethod

GOLDEN_DIR = Path(__file__).resolve().parent

# Senales built-in de QObject: presentes en toda instancia, no forman parte
# de los contratos de dominio, se filtran para mantener el golden estable.
QOBJECT_BUILTIN_SIGNALS = {"destroyed", "objectNameChanged"}


def manager_classes():
    """Importa las clases de senales actuales y las devuelve en una lista fija.

    La lista define el orden y las claves del golden. Tras el refactor de la
    Fase 1 este punto es el unico que se adapta (mapeando la nueva API del bus
    a los mismos nombres de clase), y el golden NO se regenera.
    """
    from src.services.data.signals import (
        CameraSignalManager,
        ConfigSignalManager,
        KinematicsSignalManager,
        PhysicalSignalManager,
        PickPlaceSignalManager,
        SearchSignalManager,
        SimulationSignalManager,
        SlidersSignalManager,
        ThemeSignalManager,
        _SignalManager,
    )

    return [
        _SignalManager,
        CameraSignalManager,
        ConfigSignalManager,
        KinematicsSignalManager,
        PhysicalSignalManager,
        PickPlaceSignalManager,
        SearchSignalManager,
        SimulationSignalManager,
        SlidersSignalManager,
        ThemeSignalManager,
    ]


def enum_classes():
    """Importa los enums con contrato estable (Fase 3)."""
    from src.services.data.enums.types import Domains, Modes, NotificationType, Units
    from src.features.pick_and_place.pick_place_states import PickPlaceState

    return [Modes, Units, Domains, NotificationType, PickPlaceState]


def collect_signals(cls):
    """Snapshot de senales de una clase via metaObject.

    Devuelve {nombre_senal: [tipos_cpp_parametros]} incluyendo las senales
    heredadas de la MRO propia (p.ej. _SignalManager) pero excluyendo las
    built-in de QObject.
    """
    obj = cls()
    try:
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
    finally:
        del obj


def collect_enum(cls):
    """Snapshot de miembros de un Enum: {miembro: valor}."""
    return {m.name: m.value for m in cls}


def snapshot_all():
    """Snapshot completo de managers y enums como diccionario serializable."""
    managers = {c.__name__: collect_signals(c) for c in manager_classes()}
    enums = {c.__name__: collect_enum(c) for c in enum_classes()}
    return {"managers": managers, "enums": enums}


def load_golden():
    """Carga el golden commiteado como diccionario."""
    import json

    return json.loads((GOLDEN_DIR / "golden_snapshots.json").read_text(encoding="utf-8"))
