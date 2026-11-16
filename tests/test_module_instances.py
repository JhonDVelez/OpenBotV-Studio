"""Tests de contratos de las instancias de módulo (Fase 4).

Pinan el comportamiento de la superficie nueva (equivalente al
comportamiento previo, verificado primero por caracterización):
- Identidad de instancia única a través de rutas de importación
- Sin ceremonia de singleton (get_instance eliminado)
- Cadencias de tick de frame_counter y global_timer
- Acceso al puerto serie (serial_ports) y notificaciones

Convenciones: Qt corre offscreen (ver tests/conftest.py).
"""

import importlib

import pytest

from src.services.data.timers import (
    GlobalTimer, global_timer, FrameCounter, frame_counter)
from src.services.data.enums.types import NotificationType
from src.services.robot import serial_manager
from src.services.robot.serial_manager import SerialPortManager, serial_ports
from src.services.ui.notification_manager import (
    NotificationManager, notification_manager)
from src.services.data.signals import (
    DrawViewSignalManager, draw_view)


# ---------------------------------------------------------------------------
# Instancias únicas de módulo
# ---------------------------------------------------------------------------

class TestModuleInstanceIdentity:
    def test_frame_counter_is_module_instance(self):
        assert isinstance(frame_counter, FrameCounter)
        assert importlib.import_module(
            "src.services.data.timers.frame_counter").frame_counter is frame_counter
        assert importlib.import_module(
            "src.services.data.timers").frame_counter is frame_counter
        assert not hasattr(FrameCounter, "get_instance")

    def test_global_timer_is_module_instance(self):
        assert isinstance(global_timer, GlobalTimer)
        assert importlib.import_module(
            "src.services.data.timers.global_timer").global_timer is global_timer
        assert importlib.import_module(
            "src.services.data.timers").global_timer is global_timer
        assert not hasattr(GlobalTimer, "get_instance")

    def test_serial_ports_is_module_instance(self):
        assert isinstance(serial_ports, SerialPortManager)
        assert importlib.import_module(
            "src.services.robot.serial_manager").serial_ports is serial_ports
        assert not hasattr(SerialPortManager, "get_instance")

    def test_notification_manager_is_module_instance(self):
        assert isinstance(notification_manager, NotificationManager)
        assert importlib.import_module(
            "src.services.ui.notification_manager").notification_manager is notification_manager
        assert not hasattr(NotificationManager, "get_instance")

    def test_draw_view_is_module_instance(self):
        assert isinstance(draw_view, DrawViewSignalManager)
        assert importlib.import_module(
            "src.services.data.signals.draw_view").draw_view is draw_view
        assert not hasattr(DrawViewSignalManager, "get_instance")

    def test_notification_manager_instantiable_now(self):
        # La guardia RuntimeError del singleton ya no existe
        fresh = NotificationManager()
        assert fresh is not notification_manager


# ---------------------------------------------------------------------------
# frame_counter: cadencia de emisión
# ---------------------------------------------------------------------------

class TestFrameCounterCadence:
    def test_tick_emits_every_interval(self, qtbot):
        emissions = []
        frame_counter.process_frame_signal.connect(
            lambda: emissions.append(1))

        # Intervalo determinista para el test (set_interval persistiría config)
        frame_counter._interval = 3
        frame_counter.reset()
        try:
            for _ in range(7):
                frame_counter.tick()
        finally:
            frame_counter.process_frame_signal.disconnect()
            frame_counter._interval = 4
            frame_counter.reset()

        # Emite en los ticks 3 y 6; el tick 7 reinicia el contador
        assert emissions == [1, 1]

    def test_reset_zeroes_counter(self, qtbot):
        frame_counter.reset()
        emissions = []
        frame_counter.process_frame_signal.connect(
            lambda: emissions.append(1))
        try:
            for _ in range(2):  # < intervalo por defecto
                frame_counter.tick()
        finally:
            frame_counter.process_frame_signal.disconnect()

        assert emissions == []

    def test_set_interval_updates_interval(self, qtbot):
        emissions = []
        frame_counter.process_frame_signal.connect(
            lambda: emissions.append(1))
        frame_counter.reset()
        try:
            frame_counter.set_interval(2)
            for _ in range(5):
                frame_counter.tick()
        finally:
            frame_counter.process_frame_signal.disconnect()
            frame_counter.reset()

        # set_interval(2) también resetea el contador: emite en ticks 2 y 4
        assert emissions == [1, 1]


# ---------------------------------------------------------------------------
# global_timer: distribución de ticks
# ---------------------------------------------------------------------------

class TestGlobalTimerDistribution:
    def test_tick_distribution(self, qtbot):
        counts = {"model": 0, "sync_sim": 0, "sync_robot": 0, "update": 0}
        global_timer.model_tick.connect(
            lambda: counts.__setitem__("model", counts["model"] + 1))
        global_timer.sync_simulation_tick.connect(
            lambda: counts.__setitem__("sync_sim", counts["sync_sim"] + 1))
        global_timer.sync_robot_tick.connect(
            lambda: counts.__setitem__("sync_robot", counts["sync_robot"] + 1))
        global_timer.update_tick.connect(
            lambda: counts.__setitem__("update", counts["update"] + 1))

        # Estado determinista
        global_timer._sync_counter = 0
        global_timer._model_counter = 0
        try:
            for _ in range(30):
                global_timer._tick()
        finally:
            global_timer.model_tick.disconnect()
            global_timer.sync_simulation_tick.disconnect()
            global_timer.sync_robot_tick.disconnect()
            global_timer.update_tick.disconnect()

        # model cada 4 ticks (4,8,...,28) = 7; sync en tick 25 = 1;
        # update en todos los demás = 29
        assert counts["model"] == 7
        assert counts["sync_robot"] == 1
        assert counts["sync_sim"] == 1
        assert counts["update"] == 29

    def test_start_stop_idempotent(self, qtbot):
        try:
            global_timer.start()
            assert global_timer._timer.isActive()
            global_timer.start()  # idempotente
            assert global_timer._timer.isActive()
            global_timer.stop()
            assert not global_timer._timer.isActive()
        finally:
            global_timer.stop()


# ---------------------------------------------------------------------------
# serial_ports: acceso sin hardware
# ---------------------------------------------------------------------------

class TestSerialPortsAccess:
    def test_request_access_invalid_port_fails(self):
        result = serial_ports.request_access(
            "COM_NO_EXISTE_XYZ_9999", "TestOwner")
        assert result is False
        assert serial_ports.get_serial() is None
        assert not serial_ports.is_active("TestOwner")

    def test_release_unknown_owner_is_noop(self):
        serial_ports.release_access("OwnerInexistente")  # no debe lanzar
        assert serial_ports.get_serial() is None

    def test_safe_io_without_serial(self):
        assert serial_ports.safe_readline() == b""
        serial_ports.safe_write(b"data")  # no debe lanzar
        assert serial_ports.get_in_waiting() == 0


# ---------------------------------------------------------------------------
# notification_manager: notificaciones
# ---------------------------------------------------------------------------

class TestNotificationManager:
    def test_set_main_window(self, qtbot):
        notification_manager.set_main_window(None)
        assert notification_manager._main_window is None

    def test_toast_notify_returns_none(self, qtbot):
        notification_manager.set_main_window(None)
        result = notification_manager.notify(
            "Test toast", NotificationType.TOAST_INFO)
        # TOAST types no devuelven nada
        assert result is None


# ---------------------------------------------------------------------------
# draw_view: estado de overlays
# ---------------------------------------------------------------------------

class TestDrawViewState:
    def test_setters_and_getters(self):
        try:
            draw_view.set_charuco(True)
            draw_view.set_circle(False)
            assert draw_view.get_state() == (True, False)

            draw_view.set_circle(True)
            assert draw_view.get_state() == (True, True)
        finally:
            draw_view.set_charuco(False)
            draw_view.set_circle(False)
