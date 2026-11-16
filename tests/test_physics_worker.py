"""
Tests de caracterización del bucle de física de PhysicsWorker.

El worker ejecuta PyBullet en su propio hilo a 240 Hz (1/240 s por paso),
emite pose al QML cada 4 pasos (60 Hz) y telemetría a gráficas cada 24
pasos (10 Hz). RobotArmPhysics se sustituye por un fake para no cargar
el motor de física real.
"""

import pytest

import src.services.simulation.physics_worker as pw


class FakePhysics:
    """Doble de RobotArmPhysics: registra llamadas y devuelve pose fija."""

    def __init__(self):
        self.joint_indices = list(range(6))
        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_calls = []
        self.step_count = 0
        self.released = False
        self.sphere_calls = []

    def load_models(self, robot_id):
        pass

    def set_joint_positions(self, positions, max_velocity=1.2):
        self.set_calls.append(list(positions))

    def get_joint_positions(self):
        return list(self.pose)

    def step_simulation(self):
        self.step_count += 1

    def has_released_spheres(self):
        return self.released

    def get_sphere_position(self):
        return {}

    def update_spheres(self, poses):
        self.sphere_calls.append(("spheres", poses))

    def release_sphere(self, color):
        self.sphere_calls.append(("release", color))

    def reattach_sphere(self, color):
        self.sphere_calls.append(("reattach", color))

    def hide_all_spheres(self):
        self.sphere_calls.append(("hide_all",))

    def update_sphere_scale(self, radius):
        self.sphere_calls.append(("scale", radius))


@pytest.fixture
def fake_physics(monkeypatch):
    fake = FakePhysics()
    monkeypatch.setattr(pw, "RobotArmPhysics", lambda: fake)
    return fake


# Objetivo con índices 4 y 5 no nulos: si el worker pasara la lista por
# referencia, set_joint_positions la negaría in-place y se detectaría aquí.
TARGET = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_update_target_encola_y_da_un_paso_canonico(qapp, fake_physics):
    w = pw.PhysicsWorker("robot")

    w.update_target(TARGET)
    w._drain_commands()
    w._step()

    assert fake_physics.set_calls == [TARGET]
    assert w.target_position == TARGET
    assert fake_physics.step_count == 1


def test_comandos_de_esferas_se_ejecutan_en_el_drenaje(qapp, fake_physics):
    w = pw.PhysicsWorker("robot")

    w.update_sphere_initial_positions({"amarillo": [1, 2, 3]})
    w.release_sphere("amarillo")
    w.reattach_sphere("amarillo")
    w.hide_all_spheres()
    w.update_sphere_scale(25.0)
    w._drain_commands()

    assert fake_physics.sphere_calls == [
        ("spheres", {"amarillo": [1, 2, 3]}),
        ("release", "amarillo"),
        ("reattach", "amarillo"),
        ("hide_all",),
        ("scale", 25.0),
    ]


def test_emite_pose_cada_4_pasos_y_sensores_cada_24(qapp, fake_physics):
    w = pw.PhysicsWorker("robot")
    received_model = []
    received_sensor = []
    w.model_updated.connect(lambda *a: received_model.append(a))
    w.sensor_updated.connect(lambda *a: received_sensor.append(a))

    for _ in range(3):
        w._step()
    assert received_model == [] and received_sensor == []

    w._step()  # 4º paso
    assert len(received_model) == 1
    assert received_model[0][0] == fake_physics.pose
    assert received_sensor == []

    for _ in range(20):
        w._step()  # llega al paso 24
    assert len(received_model) == 6
    assert len(received_sensor) == 1


def test_paso_quieto_no_avanza_fisica(qapp, fake_physics):
    w = pw.PhysicsWorker("robot")
    w._drain_commands()
    w._step()
    # Sin objetivo distinto y sin esferas liberadas: no hay stepSimulation.
    assert fake_physics.step_count == 0


def test_esfera_liberada_avanza_fisica_con_brazo_quieto(qapp, fake_physics):
    w = pw.PhysicsWorker("robot")
    fake_physics.released = True
    w._drain_commands()
    w._step()
    assert fake_physics.step_count == 1
