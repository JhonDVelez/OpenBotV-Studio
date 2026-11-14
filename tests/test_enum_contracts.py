"""Contratos de los enums del sistema (Fase 3 del refactor).

Los enums son parte del contrato publico: sus nombres de miembro y valores
viajan por senales, se persisten en config JSON y se comparan con hardware.
Cualquier cambio debe ser intencional y actualizar el golden a la vez.
"""

import pytest

from tests.golden.registry import collect_enum, enum_classes, load_golden


@pytest.mark.usefixtures("qapp")
def test_enum_values_match_golden():
    golden = load_golden()["enums"]
    current = {c.__name__: collect_enum(c) for c in enum_classes()}

    missing = set(golden) - set(current)
    added = set(current) - set(golden)
    assert not missing, f"Enums desaparecidos: {sorted(missing)}"
    assert not added, f"Enums nuevos: {sorted(added)}"

    for name in golden:
        assert current[name] == golden[name], (
            f"Enum {name} cambio:\n  golden: {golden[name]}\n  actual: {current[name]}"
        )


@pytest.mark.usefixtures("qapp")
def test_enum_members_are_unique_and_hashable():
    for cls in enum_classes():
        members = list(cls)
        names = [m.name for m in members]
        assert len(names) == len(set(names)), f"{cls.__name__}: miembros duplicados"
        for m in members:
            hash(m)  # usable como clave de dict / en sets
