"""
Paquete de simulación del brazo robótico con PyBullet.

Gestiona la simulación física del modelo 3D, el entorno de PyBullet,
el envío y recepción de datos de este, las mallas y texturas del
modelo visible en la interfaz, y el archivo URDF empleado por PyBullet.

Señales:
    - bus.sim_update_pybullet_signal: Actualiza posiciones
      objetivo de la simulación.
    - PhysicsWorker.model_updated: Pose a 60 Hz para el modelo 3D.
    - PhysicsWorker.sensor_updated: Telemetría a 10 Hz para las gráficas.
"""

from .physics_worker import PhysicsWorker

__all__ = [
    "PhysicsWorker"
]
