"""
Módulo del worker de física del brazo robótico en PyBullet.

Proporciona la clase PhysicsWorker, que ejecuta el bucle de simulación en
su propio hilo: avanza PyBullet a 240 Hz (un stepSimulation cada 1/240 s
de tiempo real; el dt ya lo fija el preloader con setTimeStep), emite la
pose al modelo 3D QML a 60 Hz y los datos de gráficas a 10 Hz.

Los comandos que tocan PyBullet (objetivos, esferas, escala) llegan desde
el hilo principal vía una cola y solo se ejecutan dentro del bucle, de
forma que PyBullet vive en un único hilo.

Conexiones:
    - Conectado a bus.sim_update_pybullet_signal
      para recibir posiciones objetivo.
    - Emite posiciones a traves del bus (señales sim_)
      (model_position_signal, sensor_position_signal).
"""

import queue
import time
import traceback

from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QApplication
from src.services.simulation.physics_pybullet import RobotArmPhysics


class PhysicsWorker(QThread):
    """    Worker encargado de actualizar la simulación de PyBullet en segundo plano.

    Gestiona la sincronización entre las posiciones objetivo, la simulación
    física y la representación 3D mostrada en la interfaz.

    Args:
        robot_id: Identificador del robot cargado en PyBullet.
    """

    # Cadencias del bucle: 240 Hz de física, 60 Hz de pose y 10 Hz de gráficas.
    STEP_HZ = 240.0
    POSE_EVERY = 4
    SENSOR_EVERY = 24
    PAUSE_SLEEP_S = 0.005

    # Señales locales para comunicación con el controlador
    # (Evita el uso de SignalManagers globales en el worker)
    model_updated = pyqtSignal(list, dict)
    sensor_updated = pyqtSignal(list)

    def __init__(self, robot_id) -> None:
        super().__init__()
        self.target_position = [0, 0, 0, 0, 0, 0]
        self.target_position_prev = [1, 0, 0, 0, 0, 0]
        self.physic = None
        self.max_velocity = 1.5

        self.physic = RobotArmPhysics()
        self.physic.load_models(robot_id)

        self._commands = queue.SimpleQueue()
        self._running = False
        self._quit = False
        self._tick = 0
        self._last_pose = None
        # Último objetivo realmente aplicado, en espacio PyBullet
        # (índices 4 y 5 negados): la pose leída se compara contra esto.
        self._applied = [0.0] * 6

        # El hilo es dueño de su muerte: cualquier salida por el event loop
        # de Qt lo detiene aunque nadie llame a closeEvent.
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.stop)

    def set_max_velocity(self, max_vel):
        """Define la velocidad máxima de los motores en la simulación.

        Args:
            max_vel (float): Velocidad máxima en radianes por segundo.
                Debe ser mayor a 0.
        """
        if max_vel > 0:
            self.max_velocity = max_vel
        else:
            print("La velocidad máxima debe ser mayor a 0, "
                  "usando la velocidad máxima por defecto: 1 rad/s")
            self.max_velocity = 1.5

    # --- Ciclo de vida del hilo ---

    def start(self):
        """Arranca el hilo la primera vez; si ya corre, solo reactiva el bucle."""
        if self.isRunning():
            self._running = True
            return
        super().start()

    def pause(self):
        """Pausa el bucle (el hilo queda a la espera, sin busy-spin)."""
        self._running = False

    def resume(self):
        """Reanuda la simulación pausada."""
        self._running = True

    def stop(self):
        """Detiene el hilo de forma definitiva y espera su cierre."""
        self._quit = True
        self._running = False
        if self.isRunning():
            self.wait(2000)

    def run(self):
        """Ciclo principal del subproceso.

        Pacing propio: un paso de física cada 1/240 s de tiempo real. La
        deuda temporal se descarta (no se acumula) para no acelerar la
        simulación tras una pausa o un bloqueo puntual.
        """
        self._running = True
        self.set_max_velocity(1.2)
        dt = 1.0 / self.STEP_HZ
        next_t = time.perf_counter() + dt
        while not self._quit:
            try:
                self._drain_commands()
                if not self._running:
                    time.sleep(self.PAUSE_SLEEP_S)
                    next_t = time.perf_counter() + dt
                    continue
                self._step()
            except Exception:
                # Un fallo no debe matar el hilo en silencio: se registra
                # y el bucle queda estacionado; stop() sigue funcionando.
                traceback.print_exc()
                self._running = False
                continue
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
            next_t += dt

    # --- Bucle de física ---

    def _drain_commands(self):
        """Ejecuta en el hilo del worker todos los comandos pendientes."""
        while True:
            try:
                op, payload = self._commands.get_nowait()
            except queue.Empty:
                return
            if op == "target":
                self.target_position = payload
            elif op == "spheres":
                self.physic.update_spheres(payload)
            elif op == "release":
                self.physic.release_sphere(payload)
            elif op == "reattach":
                self.physic.reattach_sphere(payload)
            elif op == "hide_all":
                self.physic.hide_all_spheres()
            elif op == "scale":
                self.physic.update_sphere_scale(payload)

    def _step(self):
        """Un ciclo del bucle: objetivos, paso de física y lectura de pose."""
        if not self.physic.joint_indices:
            return
        pose = self.physic.get_joint_positions()
        if len(self.target_position) == len(self.physic.joint_indices):
            if not all(x == y for x, y in zip(self.target_position, self.target_position_prev)):
                # Copia: set_joint_positions niega in-place los índices 4 y 5.
                aplicado = list(self.target_position)
                self.physic.set_joint_positions(aplicado, self.max_velocity)
                self._applied = aplicado
                # Guardar el objetivo en espacio PyBullet (ya negado por la
                # mutación in-place) para comparar la pose leída contra él.
                self._applied = aplicado
                self.target_position_prev = list(self.target_position)
            is_moving = any(
                abs(x - y) >= 0.001
                for x, y in zip(self._applied, pose)
            )
            if is_moving or self.physic.has_released_spheres():
                self.physic.step_simulation()
                pose = self.physic.get_joint_positions()
        self._last_pose = pose
        self._tick += 1
        if self._tick % self.POSE_EVERY == 0:
            self.model_updated.emit(
                self._last_pose, self.physic.get_sphere_position())
        if self._tick % self.SENSOR_EVERY == 0:
            self.sensor_updated.emit(self._last_pose)

    # --- Entradas desde el hilo principal (thread-safe: encolar/flags) ---

    @pyqtSlot(list)
    def update_target(self, target_position):
        """Encola los ángulos objetivo de la simulación.

        Args:
            target_position (list): Lista de 6 posiciones objetivo en radianes
                (espacio angular, centrado en 0).
        """
        self._commands.put(("target", list(target_position)))

    def update_sphere_initial_positions(self, poses: dict):
        """Encola las posiciones iniciales de las esferas desde la cámara."""
        self._commands.put(("spheres", poses))

    @pyqtSlot(str)
    def release_sphere(self, color: str):
        """
        Encola la liberación de una esfera para que PyBullet use física real.

        Args:
            color (str): Identificador de la esfera seleccionada.
        """
        self._commands.put(("release", color))

    @pyqtSlot(str)
    def reattach_sphere(self, color: str):
        """
        Encola la reasociación de una esfera para que siga a la cámara.

        Args:
            color (str): Identificador de la esfera.
        """
        self._commands.put(("reattach", color))

    @pyqtSlot()
    def hide_all_spheres(self):
        """Encola la ocultación de todas las esferas no manipuladas."""
        self._commands.put(("hide_all", None))

    @pyqtSlot(float)
    def update_sphere_scale(self, radius):
        """Encola el cambio de tamaño de las esferas en PyBullet."""
        self._commands.put(("scale", radius))
