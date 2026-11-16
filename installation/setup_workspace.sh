#!/usr/bin/env bash
# Prepara únicamente archivos del proyecto. Ejecutar como usuario, sin sudo.
set -euo pipefail
TASK_INSTALL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TASK_PROJECT_ROOT="$(cd -- "$TASK_INSTALL_DIR/.." && pwd -P)"
TASK_VENV="$TASK_PROJECT_ROOT/.venv"
TASK_WS="${DANIEL_ROS2_WS:-$TASK_PROJECT_ROOT/.ros2_ws}"
TASK_SIM_DIR="$TASK_PROJECT_ROOT/external/f1tenth_gym"
TASK_SOURCE_JSON="$TASK_INSTALL_DIR/simulator-source.json"

show_help() {
  cat <<'HELP'
Uso: bash installation/setup_workspace.sh [--dependencies-only | --dry-run | --help]

Ubuntu 22.04 amd64, Python 3.10 y /opt/ros/humble. Ejecutar SIN sudo,
después de installation/install_ros2_humble.sh. Requiere acceso a PyPI y GitHub.
Prepara .venv, external/f1tenth_gym y .ros2_ws en la raíz del proyecto.
DANIEL_ROS2_WS permite usar otra carpeta absoluta para el workspace.
No borra directorios ni sustituye enlaces existentes que apunten a otra carpeta.
--dry-run muestra las operaciones previstas sin descargar ni escribir archivos.
--dependencies-only prepara Python y el motor, sin compilar los paquetes propios.
HELP
}
TASK_DEPENDENCIES_ONLY=0
case "${1:-}" in
  --dependencies-only) TASK_DEPENDENCIES_ONLY=1; shift ;;
  --help|-h) show_help; exit 0 ;;
  --dry-run)
    show_help
    printf '\nProyecto: %s\nEntorno: %s\nWorkspace: %s\n' "$TASK_PROJECT_ROOT" "$TASK_VENV" "$TASK_WS"
    printf 'Operaciones: venv --system-site-packages; pip/wheel/setuptools fijados; requirements-sim.txt; Gym 0.19; checkout verificado de simulator-source.json; colcon build.\n'
    exit 0 ;;
  '') ;;
  *) show_help >&2; exit 2 ;;
esac
[[ $# == 0 ]] || { printf 'Argumentos inesperados.\n' >&2; exit 2; }
[[ "$EUID" -ne 0 ]] || { printf 'Ejecuta este script sin sudo.\n' >&2; exit 1; }
[[ "$TASK_WS" == /* ]] || { printf 'DANIEL_ROS2_WS debe ser una ruta absoluta.\n' >&2; exit 1; }
if [[ "$TASK_PROJECT_ROOT" =~ [[:space:]] || "$TASK_WS" =~ [[:space:]] ]]; then
  printf 'Usa rutas sin espacios para el clon y el workspace, por ejemplo ~/f1tenth-sakhir. Consulta README.md.\n' >&2
  exit 1
fi
. /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(dpkg --print-architecture)" == amd64 ]] || {
  printf 'Plataforma admitida: Ubuntu 22.04 amd64.\n' >&2; exit 1;
}
python3 - <<'PY'
import sys
assert sys.version_info[:2] == (3, 10), 'Se requiere Python 3.10 de Ubuntu 22.04.'
PY
[[ -r /opt/ros/humble/setup.bash ]] || {
  printf 'Falta ROS Humble. Ejecuta primero installation/install_ros2_humble.sh con sudo.\n' >&2; exit 1;
}
for task_command in git curl ffmpeg Xvfb; do
  command -v "$task_command" >/dev/null || { printf 'Falta %s; revisa el instalador APT.\n' "$task_command" >&2; exit 1; }
done
# Impide aprovechar accidentalmente paquetes de ~/.local o un overlay anterior.
export PYTHONNOUSERSITE=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
unset PIP_TARGET PIP_PREFIX PIP_USER
export PIP_CONFIG_FILE=/dev/null
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -e "$TASK_VENV" ]]; then
  python3 -m venv --system-site-packages "$TASK_VENV"
fi
[[ -x "$TASK_VENV/bin/python" && -r "$TASK_VENV/pyvenv.cfg" ]] || {
  printf '%s existe pero no es un venv; no se reemplazará.\n' "$TASK_VENV" >&2; exit 1;
}
"$TASK_VENV/bin/python" - "$TASK_VENV" <<'PY'
import pathlib, sys
root=pathlib.Path(sys.argv[1])
assert sys.version_info[:2] == (3, 10), 'El venv existente debe usar Python 3.10.'
assert 'include-system-site-packages = true' in (root/'pyvenv.cfg').read_text(), 'El venv requiere --system-site-packages para ROS y las dependencias APT.'
PY
touch "$TASK_VENV/COLCON_IGNORE"
"$TASK_VENV/bin/python" -m pip install --disable-pip-version-check \
  'pip==22.0.2' 'setuptools==59.6.0' 'wheel==0.37.1'
"$TASK_VENV/bin/python" -m pip install --disable-pip-version-check \
  -r "$TASK_INSTALL_DIR/requirements-sim.txt"
"$TASK_VENV/bin/python" -m pip install --disable-pip-version-check \
  --no-deps --no-build-isolation 'gym==0.19.0'

TASK_SIM_URL="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["repository"])' "$TASK_SOURCE_JSON")"
TASK_SIM_COMMIT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["commit"])' "$TASK_SOURCE_JSON")"
mkdir -p "$TASK_PROJECT_ROOT/external"
touch "$TASK_PROJECT_ROOT/external/COLCON_IGNORE"
if [[ ! -e "$TASK_SIM_DIR" ]]; then
  git init --quiet "$TASK_SIM_DIR"
  git -C "$TASK_SIM_DIR" remote add origin "$TASK_SIM_URL"
  git -C "$TASK_SIM_DIR" fetch --depth 1 origin "$TASK_SIM_COMMIT"
  git -C "$TASK_SIM_DIR" checkout --detach "$TASK_SIM_COMMIT"
fi
[[ -d "$TASK_SIM_DIR/.git" ]] || { printf '%s existe pero no es el checkout esperado.\n' "$TASK_SIM_DIR" >&2; exit 1; }
[[ "$(git -C "$TASK_SIM_DIR" rev-parse HEAD)" == "$TASK_SIM_COMMIT" ]] || {
  printf 'El simulador existente no usa el commit fijado. Muévelo antes de repetir; no se modificará automáticamente.\n' >&2; exit 1;
}
[[ -z "$(git -C "$TASK_SIM_DIR" status --porcelain --untracked-files=no)" ]] || {
  printf 'Hay cambios locales en el simulador; no se sobrescribirán.\n' >&2; exit 1;
}
"$TASK_VENV/bin/python" - "$TASK_SOURCE_JSON" "$TASK_SIM_DIR" <<'PY'
import hashlib, json, pathlib, sys
spec=json.loads(pathlib.Path(sys.argv[1]).read_text())
root=pathlib.Path(sys.argv[2])
for name, expected in spec['sha256'].items():
    actual=hashlib.sha256((root/name).read_bytes()).hexdigest()
    assert actual == expected, f'El archivo del simulador no coincide: {name}'
print('Commit y SHA256 del motor verificados.')
PY
"$TASK_VENV/bin/python" -m pip install --disable-pip-version-check \
  --no-deps --no-build-isolation --editable "$TASK_SIM_DIR"

if [[ "$TASK_DEPENDENCIES_ONLY" == 1 ]]; then
  printf '\nDependencias listas. Ejecuta sin --dependencies-only cuando estén presentes ambos paquetes.\n'
  exit 0
fi

mkdir -p "$TASK_WS/src"
touch "$TASK_WS/COLCON_IGNORE"
ensure_link() {
  local task_target="$1" task_link="$2"
  if [[ -e "$task_link" || -L "$task_link" ]]; then
    [[ -L "$task_link" && "$(readlink -f -- "$task_link")" == "$task_target" ]] || {
      printf 'No se reemplazará %s: ya existe y apunta a otra ubicación.\n' "$task_link" >&2; exit 1;
    }
  else
    ln -s -- "$task_target" "$task_link"
  fi
}
ensure_link "$TASK_PROJECT_ROOT" "$TASK_WS/src/daniel_gap"
ensure_link "$TASK_PROJECT_ROOT/ros2_validation/simulator_bridge" "$TASK_WS/src/f1tenth_gym_ros"
(
  cd -- "$TASK_WS"
  "$TASK_VENV/bin/python" -m colcon build --symlink-install \
    --paths src/daniel_gap src/f1tenth_gym_ros \
    --packages-select daniel_gap f1tenth_gym_ros \
    --event-handlers console_direct+
)
mkdir -p "$TASK_PROJECT_ROOT/work/bootstrap"
"$TASK_VENV/bin/python" - "$TASK_PROJECT_ROOT/work/bootstrap/environment.json" "$TASK_SOURCE_JSON" <<'PY'
import importlib.metadata as m, json, pathlib, platform, sys
names=['pip','setuptools','wheel','numpy','scipy','numba','llvmlite','Pillow','PyYAML','pytest','gym','transforms3d','future','pyglet','PyOpenGL','cloudpickle','imageio-ffmpeg','f110-gym']
versions={name:m.version(name) for name in names}
record={'python':platform.python_version(),'executable':sys.executable,'packages':versions,'simulator':json.loads(pathlib.Path(sys.argv[2]).read_text())}
pathlib.Path(sys.argv[1]).write_text(json.dumps(record,indent=2)+'\n')
import gym, f110_gym, rclpy
print('Importaciones Gym/F1TENTH/ROS correctas.')
PY
printf '\nWorkspace listo: %s\nEjecuta ./start_ros2.sh check y después ./start_ros2.sh demo.\n' "$TASK_WS"
