# Tests — Contratos de comportamiento (OpenBotV-Studio)

Suite creada en la **Fase 0** para blindar el refactor de distribución de
clases. Los tests capturan la superficie pública **antes** de cambiar código y
re-validan la nueva API después, sin regenerar los snapshots.

## Ejecutar

```bash
# desde la raiz del repo, con el entorno .venv del proyecto
C:\Users\jvele\miniconda3\envs\.venv\python.exe -m pytest
```

Qt corre en modo *offscreen* (`QT_QPA_PLATFORM` se fija en `conftest.py`), así
que no se abre ninguna ventana.

## Qué protege cada archivo

| Archivo | Contrato |
|---|---|
| `test_signal_contracts.py` | Nombres + firmas C++ de las 93 señales de los 10 managers; emisión directa y **entre hilos** (QThread → principal); identidad de singletons; ConfigSignalManager es 100% en memoria |
| `test_enum_contracts.py` | Miembros y valores de `Modes`, `Units`, `Domains`, `NotificationType`, `PickPlaceState` |
| `test_draw_view_state.py` | Singleton thread-safe y forma del estado de overlays (los setters **no** se ejercitan: persisten en el settings.json del usuario) |

## Goldens

`golden/golden_snapshots.json` es la línea base de comportamiento generada a
partir del código **sin refactorizar**.

**Regla: no regenerar salvo cambio de contrato INTENCIONAL y revisado.**

```bash
# solo si el cambio es intencional
C:\Users\jvele\miniconda3\envs\.venv\python.exe tests/golden/generate_golden.py
```

Durante el refactor, el único punto que se adapta es
`golden/registry.py::manager_classes()` (mapea la nueva API del `SignalBus` a
los nombres de clase del golden); los tests deben seguir en verde sin tocar el
JSON.
