"""Tests de contratos de las clases ligeras refactorizadas (Fase 3).

Pinan el comportamiento de la superficie nueva (equivalente al
comportamiento previo, verificado primero por caracterización):
- PickPlaceContext: dataclass de estado (defaults y reset)
- apply_physical_limits: función plana en robot_limits.py
- run_circle_detection: función plana de detección de esferas
- servo_helpers: relative_to_servo / with_gripper (vía PickExecutor)

Convenciones: Qt corre offscreen (ver tests/conftest.py).
"""

import importlib

import numpy as np
import cv2
import pytest

from src.features.pick_and_place.logic.context import PickPlaceContext
from src.features.pick_and_place.logic.pick_executor import PickExecutor
from src.services.robot import robot_limits
from src.services.robot.robot_limits import apply_physical_limits
from src.services.data.utils.conversions import (
    angulos_robotang, robotang_angulos)
from src.services.vision.circle_detection import run_circle_detection, COLORES


# ---------------------------------------------------------------------------
# PickPlaceContext (dataclass)
# ---------------------------------------------------------------------------

class TestPickPlaceContext:
    def test_defaults(self):
        ctx = PickPlaceContext()
        assert ctx.selected_color is None
        assert ctx.tamano_seleccionado == 30
        assert ctx.sphere_poses == {}
        assert ctx.place_target_coords is None
        assert ctx.ik_target is None
        assert ctx.gripper_open == -112.0
        assert ctx.gripper_closed == 7.0
        assert ctx.gripper_max_closed == 21.0
        assert ctx.last_positions is None
        assert ctx.current_feedback is None
        assert ctx.current_target is None

    def test_reset_clears_state_but_keeps_sphere_poses_and_gripper(self):
        ctx = PickPlaceContext()
        ctx.selected_color = "amarillo"
        ctx.tamano_seleccionado = 45
        ctx.sphere_poses = {"amarillo": {"position": (1, 2, 3)}}
        ctx.place_target_coords = {"x": 1, "y": 2, "z": 3}
        ctx.ik_target = [10, 20, 30]
        ctx.last_positions = [1, 2, 3, 4, 5, 6]
        ctx.current_feedback = [9, 9, 9, 9, 9, 9]

        ctx.reset()

        assert ctx.selected_color is None
        assert ctx.tamano_seleccionado == 30
        assert ctx.place_target_coords is None
        assert ctx.ik_target is None
        assert ctx.last_positions is None
        assert ctx.current_feedback is None
        # Se conservan
        assert ctx.sphere_poses == {"amarillo": {"position": (1, 2, 3)}}
        assert ctx.gripper_open == -112.0
        assert ctx.gripper_closed == 7.0
        assert ctx.gripper_max_closed == 21.0
        assert ctx.current_target is None

    def test_is_dataclass_with_field_defaults(self):
        import dataclasses
        assert dataclasses.is_dataclass(PickPlaceContext)
        names = {f.name for f in dataclasses.fields(PickPlaceContext)}
        assert {"selected_color", "tamano_seleccionado", "sphere_poses",
                "place_target_coords", "ik_target", "gripper_open",
                "gripper_closed", "gripper_max_closed", "last_positions",
                "current_feedback", "current_target"} <= names
        # Los umbral muertos de la versión anterior no deben existir
        assert not names & {"STALL_TIMEOUT_MS", "MOVEMENT_TOLERANCE",
                            "SUCCESS_THRESHOLD", "ERROR_THRESHOLD"}


# ---------------------------------------------------------------------------
# robot_limits (ex CartesianPidCompensator)
# ---------------------------------------------------------------------------

class TestRobotLimits:
    def test_compensator_class_is_gone(self):
        with pytest.raises(ImportError):
            importlib.import_module("src.services.robot.robot_compensator")

    def test_apply_physical_limits_within_limits_unchanged(self):
        q = np.array([0.0, 0.0, 0.0, 0.0])
        result = apply_physical_limits(q)
        assert np.allclose(result, q)
        assert isinstance(result, np.ndarray)

    def test_apply_physical_limits_clamps(self):
        # Límite eje 1: -100..100 deg
        q = np.array([np.radians(120.0), 0.0, 0.0, 0.0])
        result = apply_physical_limits(q)
        assert np.isclose(result[0], np.radians(100.0))

        # Límite eje 4: -90..120 deg (por debajo)
        q2 = np.array([0.0, 0.0, 0.0, np.radians(-120.0)])
        result2 = apply_physical_limits(q2)
        assert np.isclose(result2[3], np.radians(-90.0))

    def test_apply_physical_limits_custom_limits(self):
        custom = [(0.0, 10.0), (0.0, 10.0), (0.0, 10.0), (0.0, 10.0)]
        q = np.array([np.radians(50.0), 0.0, 0.0, 0.0])
        result = apply_physical_limits(q, custom)
        assert np.isclose(result[0], np.radians(10.0))

    def test_conversions_functions_unchanged(self):
        assert angulos_robotang(10, -10, 20, 30, 40, 50) == [
            160, 160, 130, 180, 190, 200]
        assert robotang_angulos(160, 160, 130, 180, 190, 200) == [
            10, -10, 20, 30, 40, 50]


# ---------------------------------------------------------------------------
# Servo helpers (ex BaseExecutor) vía PickExecutor
# ---------------------------------------------------------------------------

class _DummyWorker:
    pass


class TestServoHelpers:
    def test_relative_to_servo(self):
        executor = PickExecutor(_DummyWorker(), PickPlaceContext())
        assert executor.worker is not None and executor.context is not None
        from src.features.pick_and_place.logic.servo_helpers import (
            relative_to_servo, with_gripper)

        assert relative_to_servo([0, 0, 0, 0, 90, 0]) == [
            150.0, 150.0, 150.0, 150.0, 240.0, 150.0]
        assert all(isinstance(v, float) for v in
                   relative_to_servo([0, 0, 0, 0, 90, 0]))

    def test_with_gripper(self):
        from src.features.pick_and_place.logic.servo_helpers import (
            with_gripper)

        original = [1, 2, 3, 4, 5, 6]
        result = with_gripper(original, -112.0)
        assert result == [1, 2, 3, 4, 5, 38.0]
        assert result[-1] == float(-112.0 + 150.0)
        # original no mutado
        assert original == [1, 2, 3, 4, 5, 6]

    def test_base_executor_is_gone(self):
        with pytest.raises(ImportError):
            importlib.import_module(
                "src.features.pick_and_place.logic.base_executor")

    def test_executors_do_not_inherit_base_class(self):
        assert PickExecutor.__bases__ == (object,)


# ---------------------------------------------------------------------------
# run_circle_detection (ex CircleDetection QRunnable)
# ---------------------------------------------------------------------------

def _make_yellow_circle_frame():
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(frame, (100, 100), 40, (0, 255, 255), -1)  # amarillo BGR
    return cv2.UMat(frame.copy())


class TestRunCircleDetection:
    def test_detects_yellow_circle(self):
        detected = []
        det_cb = lambda frame_id, results: detected.append((frame_id, results))
        errors = []
        err_cb = lambda msg: errors.append(msg)

        run_circle_detection(
            _make_yellow_circle_frame(), 7, None, None, det_cb, err_cb)

        assert not errors
        frame_id, results = detected[0]
        assert frame_id == 7
        assert "amarillo" in results
        circle = results["amarillo"]
        assert circle["circle"] is not None  # elipse ajustable (contorno >= 5 pts)
        cx, cy = circle["center"]
        assert abs(cx - 100.0) < 2.0 and abs(cy - 100.0) < 2.0
        assert 38.0 < circle["radius"] < 42.0

    def test_no_detection_calls_callback_with_none(self):
        detected = []
        det_cb = lambda frame_id, results: detected.append((frame_id, results))
        errors = []
        err_cb = lambda msg: errors.append(msg)

        empty = cv2.UMat(np.zeros((50, 50, 3), dtype=np.uint8))
        run_circle_detection(empty, 1, None, None, det_cb, err_cb)

        assert not errors
        frame_id, results = detected[0]
        assert frame_id == 1
        assert results is None

    def test_default_hsv_colors_constant(self):
        assert set(COLORES) == {"amarillo", "verde", "azul", "naranja", "morado"}
        assert COLORES["amarillo"] == (20, 100, 100, 30, 255, 255)

    def test_error_path_reports_to_error_callback(self):
        detected = []
        det_cb = lambda frame_id, results: detected.append((frame_id, results))
        errors = []
        err_cb = lambda msg: errors.append(msg)

        # hsv_colors con un valor no desempaquetable -> ValueError (capturado)
        frame = cv2.UMat(np.zeros((50, 50, 3), dtype=np.uint8))
        run_circle_detection(
            frame, 3, None, {"bad": "notatuple"}, det_cb, err_cb)

        assert not detected
        assert len(errors) == 1
        assert "Error al detectar esfera" in errors[0]
        assert "run_circle_detection" in errors[0]

    def test_qrunnable_create_integration(self, qtbot):
        """La función plana se encola en un QThreadPool vía QRunnable.create."""
        from functools import partial
        from PyQt6.QtCore import QRunnable, QThreadPool

        detected = []
        errors = []

        pool = QThreadPool()
        pool.start(QRunnable.create(partial(
            run_circle_detection,
            _make_yellow_circle_frame(), 42, None, None,
            lambda fid, res: detected.append((fid, res)),
            lambda msg: errors.append(msg))))
        pool.waitForDone(10000)

        assert not errors
        assert detected[0] == (42, detected[0][1])
        assert "amarillo" in detected[0][1]
