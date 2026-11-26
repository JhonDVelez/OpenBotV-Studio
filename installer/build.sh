#!/usr/bin/env bash
set -Eeuo pipefail

# Build portable Linux packages for OpenBotV Studio.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DIST_DIR="${ROOT}/dist/OpenBotV-Studio"
EXE_PATH="${DIST_DIR}/OpenBotV-Studio"
ARCHIVE_PATH="${ROOT}/dist/OpenBotV-Studio-portable.tar.gz"

print_usage() {
    cat <<'USAGE'
Uso:
  ./installer/build.sh             Menú interactivo
  ./installer/build.sh exe         Solo PyInstaller
  ./installer/build.sh portable    Solo archivo portátil (requiere dist)
  ./installer/build.sh all         Ejecutable + archivo portátil
  ./installer/build.sh -h          Ayuda
USAGE
}

find_python() {
    if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
        printf '%s\n' "${CONDA_PREFIX}/bin/python"
        return
    fi
    if [[ -x "${ROOT}/.venv/bin/python" ]]; then
        printf '%s\n' "${ROOT}/.venv/bin/python"
        return
    fi
    if command -v python3 >/dev/null 2>&1; then
        command -v python3
        return
    fi
    echo "No se encontró Python." >&2
    exit 1
}

PYTHON="$(find_python)"

is_dist_ready() {
    [[ -x "$EXE_PATH" ]]
}

invoke_pyinstaller() {
    echo "==> PyInstaller (onedir, Linux)"
    "$PYTHON" -m PyInstaller installer/main_linux.spec --noconfirm
    echo "OK: ${DIST_DIR}"
}

create_launcher() {
    cat > "${DIST_DIR}/OpenBotV-Studio.sh" <<'LAUNCHER'
#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export QT_QPA_PLATFORM="xcb"
exec "${SCRIPT_DIR}/OpenBotV-Studio" "$@"
LAUNCHER
    chmod 755 "${DIST_DIR}/OpenBotV-Studio.sh"

    cat > "${DIST_DIR}/OpenBotV-Studio.desktop" <<'DESKTOP'
[Desktop Entry]
Name=OpenBotV Studio
Comment=Control y simulación del brazo robótico OpenBotV
Exec=OpenBotV-Studio.sh
Terminal=false
Type=Application
Categories=Education;Science;Engineering;
Icon=OpenBotV-Studio
DESKTOP
    cp "${ROOT}/src/resources/img/laser_b.png" "${DIST_DIR}/OpenBotV-Studio.png"
}

warn_dist_stale() {
    if ! is_dist_ready; then
        return
    fi
    if [[ -n "$(find src -type f -not -path '*/__pycache__/*' -newer "$EXE_PATH" -print -quit)" ]]; then
        echo "Advertencia: dist/OpenBotV-Studio podría ser anterior al código fuente." >&2
    fi
}

invoke_portable() {
    if ! is_dist_ready; then
        echo "No existe ${EXE_PATH}. Ejecuta primero el paso exe." >&2
        exit 1
    fi
    warn_dist_stale

    echo "==> Portable (tar.gz)"
    create_launcher
    rm -f "$ARCHIVE_PATH"
    tar -C "${ROOT}/dist" -czf "$ARCHIVE_PATH" OpenBotV-Studio
    echo "OK: ${ARCHIVE_PATH}"
}

show_report() {
    echo
    echo "Artefactos en dist/:"
    ls -lh "${ROOT}/dist" | awk 'NR == 1 || /OpenBotV-Studio/'
}

TASKS=()

if [[ $# -gt 0 ]]; then
    case "$1" in
        exe) TASKS+=(exe) ;;
        portable) TASKS+=(portable) ;;
        all) TASKS+=(exe portable) ;;
        -h|--help) print_usage; exit 0 ;;
        *) echo "Tarea no válida: $1" >&2; print_usage; exit 2 ;;
    esac
else
    cat <<'MENU'

OpenBotV Studio - Build Linux
  [1] Solo PyInstaller       (regenera dist/OpenBotV-Studio)
  [2] Solo Portable          (tar.gz de dist/OpenBotV-Studio)
  [3] Todo                   (1 + 2)
  [0] Salir
MENU
    read -r -p "Elige una opción: " choice
    case "$choice" in
        1) TASKS+=(exe) ;;
        2) TASKS+=(portable) ;;
        3) TASKS+=(exe portable) ;;
        0) echo "Cancelado."; exit 0 ;;
        *) echo "Opción no válida." >&2; exit 2 ;;
    esac
fi

for task in "${TASKS[@]}"; do
    case "$task" in
        exe) invoke_pyinstaller ;;
        portable) invoke_portable ;;
    esac
done

show_report
