"""Contratos de DrawViewSignalManager (estado de overlays de la vista de camara).

Nota: pese a su nombre, esta clase NO declara senales — es un holder de estado
thread-safe (flags charuco/circle) con persistencia via ConfigSignalManager.
Se captura su contrato por separado del golden de senales.

Los setters NO se ejercitan aqui: escriben en la configuracion persistente del
usuario (settings.json) via request_change, y los tests no deben mutar config.
"""

import pytest

from src.services.data.signals import DrawViewSignalManager


@pytest.mark.usefixtures("qapp")
def test_singleton_is_thread_safe_and_stable():
    a = DrawViewSignalManager.get_instance()
    b = DrawViewSignalManager.get_instance()
    assert a is b


@pytest.mark.usefixtures("qapp")
def test_get_state_contract():
    state = DrawViewSignalManager.get_instance().get_state()
    assert isinstance(state, tuple) and len(state) == 2
    assert all(isinstance(v, bool) for v in state)
