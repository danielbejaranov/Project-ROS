#!/usr/bin/env bash
# Activa el clon y ejecuta la actividad desde cualquier carpeta.
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
activity_root="${DANIEL_F1TENTH_ROOT:-$(cd -- "$project_dir/../.." && pwd -P)}"

show_help() {
    cat <<'HELP'
Uso: ./start_ros2.sh COMANDO [argumentos]

  help       Mostrar esta ayuda.
  shell      Abrir Bash con ROS 2 y los paquetes de la actividad activos.
  doctor     Mostrar el informe de ros2 doctor --report.
  check      Comprobar dependencias, mapa y paquetes del clon.
  exec CMD   Ejecutar CMD con Python, ROS y el overlay activos.
  demo       Iniciar simulador, controlador, RViz y estado visual.
  status     Mostrar el estado del controlador; Ctrl+C termina la vista.
  <args>     Pasar cualquier otro comando a ros2, por ejemplo: topic list.

La demo se detiene con Ctrl+C en su consola. Cada inicio guarda laps.csv
en ros2_validation/demo/<fecha UTC>/, sin reemplazar los resultados previos.

El entorno principal es /opt/ros/humble + .venv + .ros2_ws del clon.
DANIEL_ROS2_WS permite elegir otro workspace; DANIEL_F1TENTH_MAP indica
el mapa sin extensión. Ejecutar installation/setup_workspace.sh primero.

La estructura original work/ se conserva como compatibilidad local.
HELP
}

command="${1:-help}"
if (( $# )); then shift; fi
case "$command" in
    help|-h|--help) show_help; exit 0 ;;
esac

native_overlay="$activity_root/work/ros_native_ws/install/local_setup.bash"
clone_workspace="${DANIEL_ROS2_WS:-$project_dir/.ros2_ws}"
clone_overlay="$clone_workspace/install/local_setup.bash"
local_environment="$activity_root/work/ros_env.sh"
local_overlay="$activity_root/work/ros_ws/install/local_setup.bash"

if [[ -f /opt/ros/humble/setup.bash && -f "$clone_overlay" && -f "$project_dir/.venv/bin/activate" ]]; then
    source /opt/ros/humble/setup.bash
    source "$project_dir/.venv/bin/activate"
    source "$clone_overlay"
    export PYTHONNOUSERSITE=1
    export DANIEL_ROS2_WS="$clone_workspace"
    printf 'Entorno del clon: ROS 2 Humble + %s/.venv + %s\n' "$project_dir" "$clone_workspace" >&2
elif [[ -n "${DANIEL_ROS2_WS:-}" ]]; then
    printf 'El workspace solicitado no está preparado: %s\nEjecuta bash installation/setup_workspace.sh\n' "$clone_workspace" >&2
    exit 1
elif [[ -f /opt/ros/humble/setup.bash && -f "$native_overlay" ]]; then
    source /opt/ros/humble/setup.bash
    source "$native_overlay"
    printf 'Entorno: ROS 2 Humble nativo + %s\n' "$native_overlay" >&2
elif [[ -f "$local_environment" && -f "$local_overlay" ]]; then
    source "$local_environment"
    source "$local_overlay"
    printf 'Entorno: ROS 2 Humble local + %s\n' "$local_overlay" >&2
else
    printf 'No se encontró un entorno ROS 2 completo en %s/work/.\n' "$activity_root" >&2
    printf 'Ejecuta bash installation/setup_workspace.sh. Consulta README.md.\n' >&2
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export DANIEL_F1TENTH_MAP="${DANIEL_F1TENTH_MAP:-$project_dir/results/Sakhir_map}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$project_dir/.runtime/ros_logs}"
export NUMBA_CACHE_DIR="${NUMBA_CACHE_DIR:-$project_dir/.runtime/numba}"
mkdir -p -- "$ROS_LOG_DIR" "$NUMBA_CACHE_DIR"

case "$command" in
    shell) exec bash --noprofile --norc -i "$@" ;;
    check) exec python3 "$project_dir/installation/check_environment.py" "$@" ;;
    exec) if (( $# == 0 )); then printf 'Falta CMD para exec.\n' >&2; exit 2; fi; exec "$@" ;;
    doctor) exec ros2 doctor --report "$@" ;;
    demo) exec ros2 launch "$project_dir/ros2_validation/live_sim.launch.py" "$@" ;;
    status) exec ros2 topic echo /daniel_gap/status --field data "$@" ;;
    ros2) exec ros2 "$@" ;;
    *) exec ros2 "$command" "$@" ;;
esac
