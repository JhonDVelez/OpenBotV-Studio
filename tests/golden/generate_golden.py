"""Genera/actualiza tests/golden/golden_snapshots.json.

Uso (desde la raiz del repo, con el entorno .venv):
    python tests/golden/generate_golden.py

REGLA: solo regenerar cuando un cambio de contrato es INTENCIONAL y revisado.
El archivo commiteado es la linea base de comportamiento previa al refactor.
"""

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

# Ejecutable como script: anadir la raiz del repo
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.golden.registry import snapshot_all  # noqa: E402


def main():
    snap = snapshot_all()
    doc = {
        "_meta": {
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "purpose": "Behavioral contract baseline captured BEFORE the class-distribution refactor (Phase 0).",
        },
        "managers": snap["managers"],
        "enums": snap["enums"],
    }
    out = Path(__file__).resolve().parent / "golden_snapshots.json"
    out.write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
    n_signals = sum(len(s) for s in snap["managers"].values())
    print(f"Golden written: {out}")
    print(f"  {len(snap['managers'])} signal manager classes, {n_signals} signal declarations")
    print(f"  {len(snap['enums'])} enums")


if __name__ == "__main__":
    main()
