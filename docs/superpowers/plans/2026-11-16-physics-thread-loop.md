# Plan: Bucle de física en hilo worker + QML a 60 Hz

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mover todo el bucle de PyBullet fuera del hilo de la GUI a 240 Hz exactos, con emisión de pose a QML a 60 Hz y telemetría a gráficas a 10 Hz.

**Architecture:** `PhysicsWorker.run()` pasa a ser un bucle real en su QThread: drena una cola de comandos (objetivos, esferas, escala), avanza física con pacing por `perf_counter` a 1/240 s, y emite pose cada 4 pasos (60 Hz) y sensores cada 24 (10 Hz). Todos los slots que tocan PyBullet pasan a encolar (GIL-safe). `SimulationController` se desconecta del `GlobalTimer`; `SimulationWorker` cachea los motores QML.

**Tech Stack:** PyQt6, PyBullet (DIRECT, `setTimeStep(1/240)` ya fijado en el preloader de `src/main.py:240`), `queue.SimpleQueue`, `time.perf_counter`.

**Spec:** diseño aprobado en conversación (Opción A) — `PhysicsWorker` con bucle real + cola de comandos; `SimulationWorker` con caché de motores QML; controlador desconectado del `GlobalTimer`. Sin dependencias nuevas.

## Global Constraints

- Interpreter: `C:\Users\jvele\miniconda3\envs\.venv\python.exe`; en shells de agente anteponer a PATH `C:\Users\jvele\miniconda3\envs\.venv\Library\bin;C:\Users\jvele\miniconda3\envs\.venv\Scripts`.
- Docstrings, comentarios y strings de UI en español (estilo del repo).
- No regenerar `tests/golden/golden_snapshots.json` (el contrato golden solo fija señales del **bus** y enums; `PhysicsWorker` no está en él — verificado en `tests/golden/registry.py`).
- Tests no mutan config persistida (`~/Documents/OpenBotVStudio/config/*.json`).
- Gotcha pytest-qt: `waitSignal` solo espera dentro de `with`.
- Firmas de señales del bus y slots `@pyqtSlot` existentes se conservan.

## Review Focus

1. **Cierre de app con simulación activa** → el hilo debe morir limpio: `stop()` + `wait(2000)`. Nota (corrige la redacción original): `SimulationController.closeEvent` NUNCA fue un hook vivo — es un `QObject` plano y nada lo llama; el cierre real pasa por `QApplication.aboutToQuit`. Implementación correcta: conectar `aboutToQuit → stop()` en `PhysicsWorker.__init__`. Verificación: test de conexión + smoke test (el smoke con `Stop-Process` NO puede probar la muerte limpia del hilo: solo prueba que la app arranca).
2. **`update_target` re-enviando la misma lista** → `set_joint_positions` muta in-place (niega índices 4,5); pasar copia para no corromper el objetivo almacenado. Test: Task 1.
3. **Targets con longitud ≠ 6 / robot sin cargar** → sin pasos ni crash (guard `joint_indices`). Test: Task 1.
4. **Pausa durante arranque / reanudar rápido** → flags atómicos, bucle durmiendo 5 ms en pausa (sin busy-spin). Verificación: smoke + revisión de código.
5. **P&P con esferas liberadas y brazo quieto** → la física sigue avanzando (`has_released_spheres`). Test: Task 1 (fake con esferas).

---

### Task 1: `PhysicsWorker` — bucle real en hilo worker + rewiring del controlador

**Files:**
- Modify: `src/services/simulation/physics_worker.py` (reescrito)
- Modify: `src/features/simulation/simulation_controller.py:71-79` (quitar 3 connects del GlobalTimer), `:328-344` (`closeEvent`: stop+wait)
- Test: `tests/test_physics_worker.py` (nuevo)

**Interfaces:**
- Consumes: `RobotArmPhysics` (API actual sin cambios: `load_models`, `set_joint_positions(positions, max_velocity)`, `get_joint_positions()`, `step_simulation()`, `has_released_spheres()`, `get_sphere_position()`, `update_spheres(dict)`, `release_sphere(str)`, `reattach_sphere(str)`, `hide_all_spheres()`, `update_sphere_scale(float)`).
- Produces: `PhysicsWorker(robot_id)` con API pública intacta — señales `model_updated(list, dict)` y `sensor_updated(list)`; métodos `start()`, `pause()`, `resume()`, **`stop()` (nuevo)**, `set_max_velocity(float)`, `update_target(list)`, `update_sphere_initial_positions(dict)`, `release_sphere(str)`, `reattach_sphere(str)`, `hide_all_spheres()`, `update_sphere_scale(float)` — todos thread-safe (encolar/flags). Internos para tests: `_drain_commands()`, `_step()`. **Eliminados:** `update_simulation()`, `update_3d_model()`, `update_graphs()`.

- [ ] **Step 1: Write the failing test** — `tests/test_physics_worker.py`, monkeypatch `RobotArmPhysics` en el namespace del módulo worker (`src.services.simulation.physics_worker`) con un fake que registre llamadas:

```python
# esencia del fake: joint_indices = list(range(6)), pose fija distinta del target
def test_update_target_encola_y_da_un_paso_canonico(qapp, monkeypatch):
    # w = PhysicsWorker("r1"); w.update_target([1,0,0,0,0,0])
    # w._drain_commands(); w._step()
    assert fake.set_calls == [[1, 0, 0, 0, 0, 0]]   # sin negación duplicada
    assert fake.step_count == 1

def test_emite_pose_cada_4_pasos_y_sensores_cada_24(qapp, monkeypatch):
    # spy por append en model_updated/sensor_updated
    # 3 pasos → sin emisión; 4º paso → model_updated(pose, {}); 24 pasos → sensor_updated
    assert len(received_model) == 1 and received_sensor == []
```

- [ ] **Step 2: Run test to verify it fails**

```powershell
$env:PATH = "C:\Users\jvele\miniconda3\envs\.venv\Library\bin;C:\Users\jvele\miniconda3\envs\.venv\Scripts;$env:PATH"
& "C:\Users\jvele\miniconda3\envs\.venv\python.exe" -m pytest tests/test_physics_worker.py -v
```
Expected: FAIL (`AttributeError: '_drain_commands'` / los slots actuales no encolan).

- [ ] **Step 3: Implement** — en `physics_worker.py`: `run()` = bucle `while not self._quit` con flags `_running/_quit` (pausa → `time.sleep(0.005)` y reset de `next_t`); pacing: `next_t += 1/240` con `perf_counter`, deuda descartada (sin acumular); `tick % 4 == 0` → `model_updated`, `tick % 24 == 0` → `sensor_updated`. `_step()` lee pose una vez por tick, fija motores solo si el objetivo cambió (pasando **copia** a `set_joint_positions`), avanza si `is_moving or has_released_spheres()`. `_drain_commands()` procesa ops `target/spheres/release/reattach/hide_all/scale` de la cola. Slots públicos = `put()` en cola o flags; `start()` con guard `isRunning()`; `stop()` = `_quit=True` + `wait(2000)`. En el controlador: borrar los 3 connects de `update_tick`/`model_tick`/`sync_simulation_tick` (mantener `model_updated`/`sensor_updated` → bus y el resto de conexiones) y añadir stop+wait en `closeEvent`.

- [ ] **Step 4: Run test to verify it passes** — mismo comando. Expected: PASS (2 tests) + suite completa verde: `& python -m pytest` (75 + 2).

- [ ] **Step 5: Commit**

```bash
git add src/services/simulation/physics_worker.py src/features/simulation/simulation_controller.py tests/test_physics_worker.py
git commit -m "refactor: bucle de física en hilo worker (240 Hz física, 60 Hz QML, 10 Hz gráficas)"
```

### Task 2: `SimulationWorker` — cachear motores QML (0 `findChild` por frame)

**Files:**
- Modify: `src/features/simulation/simulation_worker.py:85-108`
- Test: `tests/test_simulation_worker_motors.py` (nuevo)

**Interfaces:**
- Consumes: nada nuevo; `root_object` ya cargado al construirse (`init_pybullet_processing`).
- Produces: `self._motors` (lista alineada con `joint_names`); `_process_pending_update()` usa la caché (motores `None` se saltan). API pública sin cambios.

- [ ] **Step 1: Write the failing test** — árbol QObject stub (root + 6 hijos con `objectName`), `update_simulation([...])` → `_process_pending_update()` → los hijos tienen propiedad dinámica `eulerRotation`; y un contador de llamadas a `findChild` que tras la 1ª aplicación no crece:

```python
def test_aplica_rotaciones_sin_findchild_por_frame(qapp):
    # asserts: findChild_count == 6 (solo al construir) y rotaciones aplicadas
```

- [ ] **Step 2: Run** — `& python -m pytest tests/test_simulation_worker_motors.py -v`. Expected: FAIL (hoy `_process_pending_update` llama `findChild` por frame).

- [ ] **Step 3: Implement** — en `__init__` (después de asignar `joint_names`): `self._motors = [self.root_object.findChild(QObject, n) for n in self.joint_names]`; `_process_pending_update()` itera `zip(self._motors, pos_copy, self.direction_rotation)` saltando `None`.

- [ ] **Step 4: Run** — PASS + suite verde.

- [ ] **Step 5: Commit** — `git commit -m "perf: cachear motores QML en SimulationWorker (60 FPS sin findChild)"`.

### Task 3: Limpieza y verificación integral

**Files:**
- Modify: `src/features/graph/graph_controller.py:54` y `src/features/graph/graph_widget.py:172` (comentarios "(matplotlib)" → "pyqtgraph")

- [ ] **Step 1:** corregir los 2 comentarios obsoletos.
- [ ] **Step 2: Verificación completa** — `& python -m pytest` (75 + nuevos, verde), smoke test de la app en background (confirmar proceso con `MainWindowTitle: OpenBotv-Studio` a los ~20 s, luego `Stop-Process`; exit code 1 tras kill forzado es esperado) y `graphify update .`.
- [ ] **Step 3: Commit** — `git commit -m "docs: comentarios matplotlib→pyqtgraph en módulo de gráficas"`.
