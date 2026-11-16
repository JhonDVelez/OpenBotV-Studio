"""Contratos de draw_view (estado de overlays de la vista de camara).

Nota: pese al nombre de la clase, NO declara senales — es un holder de
estado thread-safe (flags charuco/circle) con persistencia via
ConfigSignalManager. Se captura su contrato por separado del golden de
senales.

Fase 4: el singleton con get_instance() fue sustituido por la instancia
de modulo ``draw_view``.

Los setters NO se ejercitan aqui: escriben en la configuracion persistente del
usuario (settings.json) via request_change, y los tests no deben mutar config.
"""

import pytest

from src.services.data.signals import DrawViewSignalManager, draw_view


@pytest.mark.usefixtures("qapp")
def test_draw_view_is_the_module_instance():
    assert isinstance(draw_view, DrawViewSignalManager)
    assert not hasattr(DrawViewSignalManager, "get_instance")


@pytest.mark.usefixtures("qapp")
def test_get_state_contract():
    state = draw_view.get_state()
    assert isinstance(state, tuple) and len(state) == 2
    assert all(isinstance(v, bool) for v in state)
