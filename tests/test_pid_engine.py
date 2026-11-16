"""
Tests del motor PID cartesiano compartido (src/services/robot/pid_engine.py).

El motor es agnóstico de la planta: recibe adaptadores de feedback y comando
inyectados. Estos tests usan una planta falsa tipo servo/PyBullet que persigue
el último comando a velocidad limitada (1.2 rad/s), porque el lazo real está
ajustado a plantas con dinámica finita: una planta instantánea desestabiliza
el término D.

Contratos verificados:
    - cinematica_directa / calcular_pseudoinversa (port exacto de los workers).
    - solve_ik_newton resuelve objetivos alcanzables y respeta límites.
    - CartesianPidEngine converge con una planta perfecta.
    - run_movement orquesta plantas múltiples (lazo activo y comando directo).
    - El abort y la pausa interrumpen el lazo sin lanzar excepciones.
"""

import math
import threading

import numpy as np
import pytest

from src.services.robot.pid_engine import (
    LINKS_DEF,
    LoopGate,
    PlantPID,
    CartesianPidEngine,
    calcular_pseudoinversa,
    cinematica_directa,
    run_movement,
    solve_ik_newton,
)

LIMITES_TARGET = [(-100, 100), (-90, 90), (-130, 130), (-90, 120)]


class FakePlant:
    """Planta articular falsa tipo servo/PyBullet.

    Persigue el último comando a velocidad limitada (como POSITION_CONTROL
    con maxVelocity), integrando en cada lectura de feedback. El lazo real
    está ajustado a plantas con dinámica finita: una planta instantánea
    desestabiliza el término D.
    """

    def __init__(self, q_deg0, ts=0.02, rate=68.8):
        self.q_deg = list(q_deg0)
        self.destino = list(q_deg0)
        self.ts = ts
        self.rate = rate  # deg/s (1.2 rad/s = 68.8 deg/s)
        self.comandos = []

    def feedback_deg(self):
        paso = np.clip(np.array(self.destino) - np.array(self.q_deg),
                       -self.rate * self.ts, self.rate * self.ts)
        self.q_deg = list(np.array(self.q_deg) + paso)
        return list(self.q_deg)

    def command(self, q_deg):
        self.destino = list(q_deg)
        self.comandos.append(list(q_deg))


def fk_deg(q_deg):
    """FK desde grados canónicos (usa las 4 articulaciones del lazo)."""
    return cinematica_directa(np.radians(
        [q_deg[0], q_deg[1], q_deg[2], q_deg[4]])).tolist()


class TestCinematicaDirecta:
    """La FK portada debe dar resultados coherentes con la geometría."""

    def test_home_da_altura_positiva(self):
        p = cinematica_directa(np.radians([0.0, -45.0, 120.0, 30.0]))
        assert p[2] > 100.0

    def test_t1_cero_proyecta_en_y_cero(self):
        p = cinematica_directa(np.radians([0.0, -45.0, 90.0, 30.0]))
        assert abs(p[1]) < 1e-9

    def test_t1_gira_la_proyeccion_en_el_plano_xy(self):
        p0 = cinematica_directa(np.radians([0.0, -45.0, 90.0, 30.0]))
        p1 = cinematica_directa(np.radians([90.0, -45.0, 90.0, 30.0]))
        assert abs(p0[0] - p1[1]) < 1e-6
        assert abs(p0[1] + p1[0]) < 1e-6


class TestCalcularPseudoinversa:
    def test_dimensiones(self):
        j_inv = calcular_pseudoinversa(np.radians([0.0, -45.0, 90.0, 30.0]))
        assert j_inv.shape == (4, 3)


class TestSolveIkNewton:
    def test_recupera_pose_alcanzable(self):
        q_objetivo = [30.0, -30.0, 60.0, 0, 40.0, 0.0]
        target = fk_deg(q_objetivo)
        q_sol = solve_ik_newton(
            target, [0.0, -45.0, 120.0, 0, 30.0, 0.0], LIMITES_TARGET)
        assert np.allclose(fk_deg(q_sol), target, atol=2.0)

    def test_devuelve_6_articulaciones(self):
        q = solve_ik_newton(
            [150.0, 0.0, 300.0], [0.0, -45.0, 120.0, 0, 30.0, 0.0],
            LIMITES_TARGET, angulo_garra=-40.0)
        assert len(q) == 6
        assert q[3] == 0
        assert q[5] == -40.0

    def test_respeta_limites(self):
        # Objetivo inalcanzable: la solución debe saturar en los límites
        q = solve_ik_newton(
            [900.0, 900.0, 900.0], [0.0, -45.0, 120.0, 0, 30.0, 0.0],
            LIMITES_TARGET)
        for i, _ in enumerate([(0,), (1,), (2,), (4,)]):
            low_deg, high_deg = LIMITES_TARGET[i]
            idx = (0, 1, 2, 4)[i]
            assert low_deg - 1e-6 <= q[idx] <= high_deg + 1e-6


class TestCartesianPidEngine:
    def test_converge_con_planta_de_velocidad_limitada(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        target = fk_deg([20.0, -30.0, 70.0, 0, 40.0, 0.0])
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            gate=LoopGate(), ts=0.02)
        ok = engine.run_to_target(target, LIMITES_TARGET, angulo_garra=0.0)
        assert ok is True
        # Dentro de las tolerancias por eje del lazo (5, 7, 5 mm)
        error = np.abs(np.array(fk_deg(plant.q_deg)) - np.array(target))
        assert error[0] < 5.0 and error[1] < 7.0 and error[2] < 5.0

    def test_emite_comandos_por_el_adaptador(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            gate=LoopGate(), ts=0.02)
        engine.run_to_target(fk_deg([10.0, -20.0, 50.0, 0, 20.0, 0.0]),
                             LIMITES_TARGET, angulo_garra=0.0)
        assert len(plant.comandos) > 0
        # Las 6 articulaciones, con q4 anclada a 0 y garra en el índice 5
        assert len(plant.comandos[0]) == 6
        assert plant.comandos[0][3] == 0
        assert plant.comandos[0][5] == 0.0

    def test_grafica_recibe_iteraciones(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        llamadas = []
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            on_iteration=lambda t, p: llamadas.append((t, p)),
            gate=LoopGate(), ts=0.02)
        engine.run_to_target(fk_deg([10.0, -20.0, 50.0, 0, 20.0, 0.0]),
                             LIMITES_TARGET, angulo_garra=0.0)
        assert len(llamadas) > 0
        assert len(llamadas[0][0]) == 3

    def test_abort_interrumpe_sin_converger(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            gate=LoopGate(), ts=0.02)
        engine.abort()
        ok = engine.run_to_target(
            [400.0, 0.0, 400.0], LIMITES_TARGET, angulo_garra=0.0)
        assert ok is False

    def test_pausa_bloquea_el_lazo(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        gate = LoopGate()
        gate.pause()
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            gate=gate, ts=0.02)
        result = {}

        def correr():
            result['ok'] = engine.run_to_target(
                fk_deg([10.0, -20.0, 50.0, 0, 20.0, 0.0]),
                LIMITES_TARGET, angulo_garra=0.0)

        hilo = threading.Thread(target=correr, daemon=True)
        hilo.start()
        gate.resume()  # el lazo estaba bloqueado hasta reanudar
        hilo.join(timeout=5.0)
        assert result.get('ok') is True

    def test_ganancias_inyectables(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        engine = CartesianPidEngine(
            feedback_deg=plant.feedback_deg, send_command=plant.command,
            gate=LoopGate(), ts=0.02)
        engine.set_gains([1.0, 1.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        ok = engine.run_to_target(fk_deg([10.0, -20.0, 50.0, 0, 20.0, 0.0]),
                                  LIMITES_TARGET, angulo_garra=0.0)
        assert ok is True


class TestRunMovement:
    """Orquestación de un movimiento sobre plantas múltiples."""

    def test_lazo_doble_converge(self):
        robot = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        sim = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        target = fk_deg([20.0, -30.0, 70.0, 0, 40.0, 0.0])
        ok = run_movement(
            target, LIMITES_TARGET, 0.0,
            plantas={
                'robot': PlantPID(robot.feedback_deg, robot.command, True),
                'sim': PlantPID(sim.feedback_deg, sim.command, True),
            },
            planta_avance='robot', ts=0.02)
        assert ok is True
        error_robot = np.abs(np.array(fk_deg(robot.q_deg)) - np.array(target))
        error_sim = np.abs(np.array(fk_deg(sim.q_deg)) - np.array(target))
        assert error_robot[0] < 5.0 and error_robot[1] < 7.0 and error_robot[2] < 5.0
        assert error_sim[0] < 5.0 and error_sim[1] < 7.0 and error_sim[2] < 5.0

    def test_sin_lazos_envia_solucion_directa_y_avanza_por_feedback(self):
        # Ambas casillas apagadas: comando directo único; el avance lo decide
        # el feedback de la planta de avance, que integra hacia el comando.
        robot = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        target = fk_deg([20.0, -30.0, 70.0, 0, 40.0, 0.0])
        ok = run_movement(
            target, LIMITES_TARGET, 0.0,
            plantas={'robot': PlantPID(robot.feedback_deg, robot.command, False)},
            planta_avance='robot', ts=0.02)
        assert ok is True
        # Un único comando directo recibido
        assert len(robot.comandos) == 1
        error = np.abs(np.array(fk_deg(robot.q_deg)) - np.array(target))
        assert error[0] < 5.0 and error[1] < 7.0 and error[2] < 5.0

    def test_planta_directa_recibe_un_comando_mientras_lazo_activo(self):
        # Casilla robot apagada (directo) y lazo del gemelo activo
        robot = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        sim = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        target = fk_deg([20.0, -30.0, 70.0, 0, 40.0, 0.0])
        ok = run_movement(
            target, LIMITES_TARGET, 0.0,
            plantas={
                'robot': PlantPID(robot.feedback_deg, robot.command, False),
                'sim': PlantPID(sim.feedback_deg, sim.command, True),
            },
            planta_avance='robot', ts=0.02)
        assert ok is True
        assert len(robot.comandos) == 1
        assert len(sim.comandos) > 1

    def test_abort_externo_detiene(self):
        plant = FakePlant([0.0, -45.0, 120.0, 0, 30.0, 0.0])
        ok = run_movement(
            fk_deg([20.0, -30.0, 70.0, 0, 40.0, 0.0]), LIMITES_TARGET, 0.0,
            plantas={'robot': PlantPID(plant.feedback_deg, plant.command, True)},
            planta_avance='robot', ts=0.02,
            should_abort=lambda: True)
        assert ok is False

    def test_sin_plantas_falla(self):
        ok = run_movement(
            [100.0, 0.0, 200.0], LIMITES_TARGET, 0.0,
            plantas={}, planta_avance='robot')
        assert ok is False
