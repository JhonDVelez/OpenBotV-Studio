"""
Motor de control PID cartesiano compartido (port de Prueba_controlv11).

Contiene el algoritmo puro —cinemática directa, Jacobiano, lazo PID y
solución de IK por Newton— desacoplado de cualquier planta concreta.
KinematicsWorker y PickAndPlaceWorker inyectan adaptadores para indicar
de dónde se lee el feedback y a dónde viaja el comando, de modo que el
mismo algoritmo gobierna al robot físico (feedback: telemetría del
microcontrolador) y al gemelo virtual (feedback: pose de PyBullet).

Espacios de coordenadas:
    - Grados canónicos: espacio del usuario/robot. Vector de 6 elementos
      [q1, q2, q3, 0, q4, garra]; el índice 3 queda anclado a 0 (motor sin
      uso en la cadena cinemática) y la FK usa los índices [0, 1, 2, 4].
    - La conversión a espacio servo (0-300) vive en
      ``src.services.data.utils.conversions`` (angulos_robotang / robotang_angulos).

Flujo de un paso del lazo (port del lazo original):
    1. Feedback (grados canónicos) -> FK -> posición cartesiana actual (mm).
    2. Dentro de tolerancias -> contador de estabilidad (10 pasos) -> fin.
    3. Feedback congelado (<0.1 mm entre pasos) -> saltar la iteración.
    4. PID (P+I+D con anti-windup y deadband) -> Jacobiano inverso ->
       saturación a límites físicos -> comando (grados canónicos).
"""

import math
import time

import numpy as np

from src.services.robot.robot_limits import apply_physical_limits

LINKS_DEF = [155.0, 92.0, 111.0, 8.0, 150.0]
GAINS_DEF = {
    "kp": np.array([0.6, 0.45, 0.495]),
    "ki": np.array([1.4522, 0.1, 0.9746]),
    "kd": np.array([0.0619, 0.01, 0.0376]),
}
TOLERANCIAS_DEF = np.array([5.0, 7.0, 5.0])
ITERACIONES_ESTABLES = 10
UMBRAL_MM = 2.5
UMBRAL_MOTOR_DEG = 0.5
TS_DEF = 0.08


# --------------------------------------------------------------------------- #
#                        CINEMÁTICA DIRECTA Y JACOBIANO                        #
# --------------------------------------------------------------------------- #

def cinematica_directa(q, links=None):
    """Cinemática directa de 4 articulaciones (radianes) -> [x, y, z] mm.

    Args:
        q: Ángulos articulares en radianes (4 valores).
        links: Longitudes de eslabón; None usa LINKS_DEF.

    Returns:
        np.ndarray: Posición del efector final [px, py, pz] en mm.
    """
    if links is None:
        links = LINKS_DEF
    t1, t2, t3, t4 = q
    L1, L2, L3, L4, L5 = links
    arg23 = t2 + t3
    arg234 = t2 + t3 + t4
    projection = (L4 * math.cos(arg23) + L3 * math.sin(arg23) +
                  L2 * math.sin(t2) + L5 * math.sin(arg234))
    px = math.cos(t1) * projection
    py = math.sin(t1) * projection
    pz = (L1 + L3 * math.cos(arg23) - L4 * math.sin(arg23)
          + L2 * math.cos(t2) + L5 * math.cos(arg234))
    return np.array([px, py, pz])


def calcular_pseudoinversa(q, links=None):
    """Pseudoinversa del Jacobiano cartesiano (4x3) para q en radianes.

    Args:
        q: Ángulos articulares en radianes (4 valores).
        links: Longitudes de eslabón; None usa LINKS_DEF.

    Returns:
        np.ndarray: Pseudoinversa del Jacobiano (4x3).
    """
    if links is None:
        links = LINKS_DEF
    t1, t2, t3, t4 = q
    L1, L2, L3, L4, L5 = links
    s1, c1 = math.sin(t1), math.cos(t1)
    s2, c2 = math.sin(t2), math.cos(t2)
    s23, c23 = math.sin(t2 + t3), math.cos(t2 + t3)
    s234, c234 = math.sin(t2 + t3 + t4), math.cos(t2 + t3 + t4)
    f = L4 * c23 + L3 * s23 + L2 * s2 + L5 * s234
    df_dt2 = -L4 * s23 + L3 * c23 + L2 * c2 + L5 * c234
    df_dt3 = -L4 * s23 + L3 * c23 + L5 * c234
    df_dt4 = L5 * c234
    dz_dt2 = -L3 * s23 - L4 * c23 - L2 * s2 - L5 * s234
    dz_dt3 = -L3 * s23 - L4 * c23 - L5 * s234
    dz_dt4 = -L5 * s234
    J = np.array([
        [-s1 * f, c1 * df_dt2, c1 * df_dt3, c1 * df_dt4],
        [c1 * f, s1 * df_dt2, s1 * df_dt3, s1 * df_dt4],
        [0, dz_dt2, dz_dt3, dz_dt4]
    ])
    return np.linalg.pinv(J)


def _rad4(q_deg6):
    """Extrae las 4 articulaciones usadas por la FK desde el vector canónico."""
    return np.radians([q_deg6[0], q_deg6[1], q_deg6[2], q_deg6[4]])


def solve_ik_newton(target_xyz, q_inicio_deg, limites_deg, angulo_garra=0.0,
                    links=None, max_iter=200, tol_mm=1.0):
    """Resuelve la cinemática inversa por Newton con pseudoinversa.

    Se usa para el caso sin casillas PID (comando directo a las plantas
    disponibles). Arranca desde la pose actual del feedback y fuerza t1 a
    apuntar al objetivo en cada paso, igual que el lazo PID.

    Args:
        target_xyz: Objetivo cartesiano [x, y, z] en mm.
        q_inicio_deg: Pose inicial en grados canónicos (vector de 6).
        limites_deg: Límites físicos (min, max) por articulación.
        angulo_garra: Ángulo de garra (índice 5 del vector canónico).
        links: Longitudes de eslabón; None usa LINKS_DEF.
        max_iter: Máximo de iteraciones de Newton.
        tol_mm: Tolerancia de convergencia en mm.

    Returns:
        list: Solución en grados canónicos (vector de 6).
    """
    links = LINKS_DEF if links is None else list(links)
    objetivo = np.asarray(target_xyz, float)
    q = np.radians([q_inicio_deg[0], q_inicio_deg[1],
                    q_inicio_deg[2], q_inicio_deg[4]], dtype=float)
    for _ in range(max_iter):
        p_actual = cinematica_directa(q, links)
        error = objetivo - p_actual
        if np.linalg.norm(error) < tol_mm:
            break
        dq = calcular_pseudoinversa(q, links) @ error
        dq_deg = np.degrees(dq)
        # Paso acotado para estabilidad numérica
        max_paso = 10.0
        factor = min(1.0, max_paso / max(float(np.max(np.abs(dq_deg))), 1e-9))
        q = q + np.radians(dq_deg * factor)
        # t1 apunta siempre al objetivo (port del lazo PID)
        q[0] = math.atan2(objetivo[1], objetivo[0])
        q = apply_physical_limits(q, limites_deg)
    d = np.degrees(q)
    return [float(d[0]), float(d[1]), float(d[2]), 0.0,
            float(d[3]), float(angulo_garra)]


# --------------------------------------------------------------------------- #
#                            PUERTA DE PAUSA/REANUDAR                          #
# --------------------------------------------------------------------------- #

class LoopGate:
    """Puerta thread-safe de pausa/reanudación del lazo de control.

    ``pause()`` bloquea a los lazos que esperen en ella; ``resume()``
    los libera y marca que el siguiente paso debe usar dt = TS (igual
    que hacía ``_t_resume_pending`` en los workers originales).
    """

    def __init__(self):
        import threading
        self._evento = threading.Event()
        self._evento.set()
        self._resume_pendiente = False

    def pause(self):
        """Bloquea el lazo."""
        self._evento.clear()

    def resume(self):
        """Reanuda el lazo y reinicia el dt del siguiente paso."""
        self._resume_pendiente = True
        self._evento.set()

    def wait(self):
        """Bloquea mientras la puerta esté cerrada."""
        self._evento.wait()

    def consumir_resume(self):
        """Retorna True una única vez tras un resume pendiente."""
        if self._resume_pendiente:
            self._resume_pendiente = False
            return True
        return False


# --------------------------------------------------------------------------- #
#                               LAZO PID CARTESIANO                            #
# --------------------------------------------------------------------------- #

class _Lazo:
    """Estado mutable de un lazo PID cartesiano contra una planta."""

    def __init__(self, gains, links):
        self.gains = gains
        self.links = links
        self.error_acumulado = np.zeros(3)
        self.error_anterior = np.zeros(3)
        self.primera_iteracion = True
        self.estabilidad = 0
        self.p_anterior = None
        self.ultimo_comando = None

    def paso(self, feedback_deg, target, tolerancias, dt, limites_deg,
             angulo_garra):
        """Un paso del lazo con el feedback recibido.

        Args:
            feedback_deg: Pose actual en grados canónicos (vector de 6).
            target: Objetivo cartesiano [x, y, z] en mm.
            tolerancias: Tolerancias [x, y, z] en mm.
            dt: Delta de tiempo del paso (s).
            limites_deg: Límites físicos por articulación.
            angulo_garra: Ángulo de garra del comando.

        Returns:
            tuple: (estado, comando, p_actual) donde estado es
                'estable' | 'congelado' | 'comando'.
        """
        q_rad = _rad4(feedback_deg)
        p_actual = cinematica_directa(q_rad, self.links)
        error_actual = np.asarray(target, float) - p_actual
        error_abs = np.abs(error_actual)
        if (error_abs[0] < tolerancias[0] and error_abs[1] < tolerancias[1]
                and error_abs[2] < tolerancias[2]):
            self.estabilidad += 1
            self.error_anterior = error_actual.copy()
            self.p_anterior = p_actual.copy()
            return "estable", None, p_actual
        self.estabilidad = 0
        if (self.p_anterior is not None
                and np.allclose(p_actual, self.p_anterior, atol=0.1)):
            self.p_anterior = p_actual.copy()
            return "congelado", None, p_actual
        self.p_anterior = p_actual.copy()
        comando = self._paso_pid(error_actual, q_rad, dt, limites_deg,
                                 target, angulo_garra)
        return "comando", comando, p_actual

    def convergio(self):
        """True si el lazo se mantuvo estable el número requerido de pasos."""
        return self.estabilidad >= ITERACIONES_ESTABLES

    def _paso_pid(self, error_actual, q_rad, dt, limites_deg, target,
                  angulo_garra):
        """Cálculo PID + Jacobiano inverso -> comando en grados canónicos."""
        dist_total = np.linalg.norm(error_actual)

        P = error_actual * self.gains["kp"]

        if dist_total < UMBRAL_MM * 2:
            self.error_acumulado *= 0.7
        else:
            self.error_acumulado += error_actual * dt
        self.error_acumulado = np.clip(self.error_acumulado, -35, 35)
        I = self.error_acumulado * self.gains["ki"]

        if self.primera_iteracion:
            D = np.zeros(3)
            self.primera_iteracion = False
        else:
            d_cruda = (error_actual - self.error_anterior) / dt
            D = d_cruda * self.gains["kd"]
        self.error_anterior = error_actual.copy()

        v_control = P + I + D

        J_inv = calcular_pseudoinversa(q_rad, self.links)
        dq = J_inv @ v_control

        dq_deg = np.degrees(dq)
        for j in range(len(dq_deg)):
            if 0 < abs(dq_deg[j]) < UMBRAL_MOTOR_DEG:
                dq_deg[j] += np.sign(dq_deg[j]) * UMBRAL_MOTOR_DEG

        q_next_rad = apply_physical_limits(q_rad + np.radians(dq_deg),
                                           limites_deg)
        q_next_rad[0] = math.atan2(target[1], target[0])
        q_out_deg = np.degrees(q_next_rad)
        comando = [float(q_out_deg[0]), float(q_out_deg[1]),
                   float(q_out_deg[2]), 0.0, float(q_out_deg[3]),
                   float(angulo_garra)]
        self.ultimo_comando = comando
        return comando


class CartesianPidEngine:
    """Lazo PID cartesiano autónomo contra una planta inyectada.

    Args:
        feedback_deg: Callable ``() -> list[6]`` con la pose actual en
            grados canónicos.
        send_command: Callable ``(q_deg6) -> None`` que envía el comando
            a la planta.
        on_iteration: Callable ``(target_xyz, pos_xyz)`` para gráficas
            (se invoca con throttle de 0.1 s).
        links: Longitudes de eslabón; None usa LINKS_DEF.
        ts: Periodo del lazo en segundos.
        gate: LoopGate opcional para pausar/reanudar el lazo.
    """

    def __init__(self, feedback_deg, send_command, on_iteration=None,
                 links=None, ts=TS_DEF, gate=None):
        self._feedback_deg = feedback_deg
        self._send_command = send_command
        self._on_iteration = on_iteration
        self.links = list(LINKS_DEF if links is None else links)
        self.ts = ts
        self.gate = gate
        self.gains = {k: v.copy() for k, v in GAINS_DEF.items()}
        self.last_commanded = None
        self._abortado = False
        self._lazo = _Lazo(self.gains, self.links)
        self._ultimo_grafico = 0.0

    def set_gains(self, kp, ki, kd):
        """Define las ganancias del lazo."""
        self.gains["kp"] = np.array(kp, dtype=np.float64)
        self.gains["ki"] = np.array(ki, dtype=np.float64)
        self.gains["kd"] = np.array(kd, dtype=np.float64)

    def reset_loop_state(self):
        """Limpia el estado del lazo (errores, estabilidad y aborto)."""
        self._lazo = _Lazo(self.gains, self.links)
        self._ultimo_grafico = 0.0
        self._abortado = False

    def abort(self):
        """Marca el aborto: la siguiente llamada a run_to_target retorna."""
        self._abortado = True

    def run_to_target(self, target_xyz, limites_deg, max_iter=800,
                      tolerancias=None, angulo_garra=0.0, t_start=None,
                      should_abort=None):
        """Ejecuta el lazo hasta converger o agotar iteraciones.

        Args:
            target_xyz: Objetivo cartesiano [x, y, z] en mm.
            limites_deg: Límites físicos por articulación.
            max_iter: Máximo de iteraciones del lazo.
            tolerancias: Tolerancias [x, y, z] en mm; None usa el defecto.
            angulo_garra: Ángulo de garra del comando.
            t_start: Marca de tiempo inicial compartida; None usa ahora.
            should_abort: Callable opcional ``() -> bool`` de aborto externo.

        Returns:
            bool: True si convergió dentro de tolerancias.
        """
        target = np.asarray(target_xyz, float)
        if tolerancias is None:
            tolerancias = TOLERANCIAS_DEF
        if t_start is None:
            t_start = time.time()
        t_anterior = t_start

        for _ in range(max_iter):
            if self._abortado or (should_abort is not None and should_abort()):
                return False
            dt_forzado = False
            if self.gate is not None:
                self.gate.wait()
                if self.gate.consumir_resume():
                    dt_forzado = True

            t_actual = time.time()
            dt = t_actual - t_anterior
            if dt < 0.01:
                dt = 0.01
            if dt_forzado:
                dt = self.ts

            estado, comando, p_actual = self._lazo.paso(
                self._feedback_deg(), target, tolerancias, dt,
                limites_deg, angulo_garra)

            if t_actual - self._ultimo_grafico >= 0.1:
                self._emitir_grafica(target, p_actual)
                self._ultimo_grafico = t_actual

            if estado == "estable" and self._lazo.convergio():
                return True
            if comando is not None:
                self._send_command(comando)
                self.last_commanded = comando

            if estado == "congelado":
                t_anterior = time.time()
            elif estado == "comando":
                t_anterior = t_actual
            time.sleep(max(0.0, self.ts - (time.time() - t_actual)))

        return False

    def _emitir_grafica(self, target, p_actual):
        if self._on_iteration is None:
            return
        self._on_iteration(target.tolist(), [float(v) for v in p_actual])


# --------------------------------------------------------------------------- #
#                     MOVIMIENTO CON MÚLTIPLES PLANTAS                          #
# --------------------------------------------------------------------------- #

class PlantPID:
    """Adaptador de una planta para ``run_movement``.

    Args:
        feedback_deg: Callable ``() -> list[6]`` pose en grados canónicos.
        send_command: Callable ``(q_deg6) -> None`` destino del comando.
        lazo_activo: True -> lazo PID cerrado contra esta planta;
            False -> recibe una única solución de IK directa.
    """

    def __init__(self, feedback_deg, send_command, lazo_activo=True):
        self.feedback_deg = feedback_deg
        self.send_command = send_command
        self.lazo_activo = lazo_activo


def run_movement(target_xyz, limites_deg, angulo_garra, plantas,
                 planta_avance, gains=None, links=None, max_iter=800,
                 tolerancias=None, ts=TS_DEF, gate=None, should_abort=None,
                 on_iteration=None, t_start=None):
    """Ejecuta un movimiento cartesiano sobre varias plantas independientes.

    Cada planta con ``lazo_activo`` corre su propio lazo PID cerrado contra
    su propio feedback (lazo ROBOT: telemetría; lazo GEMELO: pose de
    PyBullet). Las plantas sin lazo reciben una única solución de IK
    directa (Newton). El fin del movimiento lo decide la planta indicada
    en ``planta_avance``: su feedback debe mantenerse dentro de tolerancias.

    Args:
        target_xyz: Objetivo cartesiano [x, y, z] en mm.
        limites_deg: Límites físicos por articulación.
        angulo_garra: Ángulo de garra del comando.
        plantas: dict nombre -> PlantPID (solo plantas alcanzables).
        planta_avance: Nombre de la planta cuyo feedback decide el fin.
        gains: dict {"kp","ki","kd"}; None usa los defectos.
        links: Longitudes de eslabón; None usa LINKS_DEF.
        max_iter: Máximo de iteraciones del movimiento.
        tolerancias: Tolerancias [x, y, z] en mm; None usa el defecto.
        ts: Periodo del lazo en segundos.
        gate: LoopGate opcional de pausa/reanudación.
        should_abort: Callable opcional ``() -> bool`` de aborto externo.
        on_iteration: Callable ``(target_xyz, pos_xyz)`` para gráficas,
            alimentado por la planta de avance (throttle 0.1 s).
        t_start: Marca de tiempo inicial compartida; None usa ahora.

    Returns:
        bool: True si la planta de avance convergió dentro de tolerancias.
    """
    if not plantas:
        return False
    target = np.asarray(target_xyz, float)
    if tolerancias is None:
        tolerancias = TOLERANCIAS_DEF
    gains = GAINS_DEF if gains is None else gains
    links = LINKS_DEF if links is None else list(links)
    if t_start is None:
        t_start = time.time()

    lazos = {nombre: _Lazo(gains, links)
             for nombre, pl in plantas.items() if pl.lazo_activo}

    # Solución directa (IK Newton) para las plantas sin lazo: un solo envío
    fb_avance = plantas[planta_avance].feedback_deg()
    if fb_avance is None:
        return False
    q_directo = solve_ik_newton(target, fb_avance, limites_deg,
                                angulo_garra, links)
    for nombre, pl in plantas.items():
        if not pl.lazo_activo:
            pl.send_command(q_directo)

    est_avance = 0
    ultimo_grafico = 0.0
    t_anterior = t_start

    for _ in range(max_iter):
        if should_abort is not None and should_abort():
            return False
        dt_forzado = False
        if gate is not None:
            gate.wait()
            if gate.consumir_resume():
                dt_forzado = True

        t_actual = time.time()
        dt = t_actual - t_anterior
        if dt < 0.01:
            dt = 0.01
        if dt_forzado:
            dt = ts

        # Avance: el feedback de la planta que decide el fin del movimiento
        fb_av = plantas[planta_avance].feedback_deg()
        if fb_av is None:
            return False
        p_av = cinematica_directa(_rad4(fb_av), links)
        err_av = target - p_av
        eabs_av = np.abs(err_av)
        if t_actual - ultimo_grafico >= 0.1:
            if on_iteration is not None:
                on_iteration(target.tolist(), [float(v) for v in p_av])
            ultimo_grafico = t_actual
        if (eabs_av[0] < tolerancias[0] and eabs_av[1] < tolerancias[1]
                and eabs_av[2] < tolerancias[2]):
            est_avance += 1
            if est_avance >= ITERACIONES_ESTABLES:
                return True
        else:
            est_avance = 0

        # Un paso del lazo propio de cada planta con PID activo
        comando_alguno = False
        congelado_alguno = False
        for nombre, pl in plantas.items():
            if not pl.lazo_activo:
                continue
            lazo = lazos[nombre]
            fb = pl.feedback_deg()
            if fb is None:
                continue
            estado, comando, _ = lazo.paso(
                fb, target, tolerancias, dt, limites_deg, angulo_garra)
            if estado == "comando":
                comando_alguno = True
                pl.send_command(comando)
            elif estado == "congelado":
                congelado_alguno = True

        # Libro de tiempos (port de la semántica original del lazo simple)
        if comando_alguno:
            t_anterior = t_actual
        elif congelado_alguno:
            t_anterior = time.time()

        time.sleep(max(0.0, ts - (time.time() - t_actual)))

    return False
