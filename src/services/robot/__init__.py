"""
Paquete de servicios de comunicación y control del robot.

Proporciona la clase RobotController (interfaz principal) y
RobotWorker (comunicación serial). Las utilidades de limitación
física viven en robot_limits.py y las conversiones angulo-servo en
src.services.data.utils.conversions.

Señales:
    - RobotWorker.data_received: Emitida al recibir telemetría válida
      desde la placa OpenCM9.04.
    - bus.phy_is_connected: Estado de la conexión serial.
    - bus.phy_data_received: Datos de telemetría recibidos.
"""

from .robot_controller import RobotController

__all__ = ['RobotController']
