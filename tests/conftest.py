"""Fixtures compartidas de la suite de tests de OpenBotV-Studio.

Configura la importacion del paquete `src` y ejecuta Qt en modo offscreen
para poder crear QObjects/QWidgets sin abrir ventanas reales.
"""

import os
import sys
from pathlib import Path

# Repo root en sys.path para `from src...` en cualquier runner (IDE, CI, CLI)
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Renderizado sin ventana real: valido tambien en CI headless
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
