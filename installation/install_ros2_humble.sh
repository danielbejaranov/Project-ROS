#!/usr/bin/env bash
# Instalación del sistema; Ubuntu 22.04 amd64. No instala paquetes con pip.
set -euo pipefail
TASK_INSTALL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK_LOG_DIR="/var/log/daniel-f1tenth"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Uso: sudo bash installation/install_ros2_humble.sh\nUbuntu 22.04 amd64; instala ROS 2 Humble y dependencias APT. Registro: /var/log/daniel-f1tenth. Después ejecuta installation/setup_workspace.sh sin sudo.\n'
  exit 0
fi
[[ $# == 0 ]] || { printf 'Argumento desconocido: %s\n' "$1" >&2; exit 2; }
if [[ "$EUID" -ne 0 ]]; then
  printf 'Ejecuta este instalador con sudo: sudo bash %q\n' "$TASK_INSTALL_DIR/install_ros2_humble.sh" >&2
  exit 1
fi
mkdir -p "$TASK_LOG_DIR"
exec > >(tee -a "$TASK_LOG_DIR/install.log") 2>&1
trap 'printf "failed\n" > "$TASK_LOG_DIR/state"' ERR
. /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(dpkg --print-architecture)" == amd64 ]]
printf 'installing\n' > "$TASK_LOG_DIR/state"
printf '\nInstalación ROS 2 Humble: %s\n' "$(date --iso-8601=seconds)"
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends --no-remove install \
  ca-certificates curl gnupg software-properties-common
# ROS Humble requiere Universe en Jammy.
add-apt-repository -y universe
TASK_REPO_DEB="$TASK_LOG_DIR/ros2-apt-source_1.3.0.jammy_all.deb"
curl --fail --location --retry 3 --output "$TASK_REPO_DEB" \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb
python3 - "$TASK_REPO_DEB" <<'PY'
import hashlib,sys
from pathlib import Path
expected='110b9a462d55252decb8b7c816f61c2ba0d9890ce5fb93ac504e97cae5860d76'
assert hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest()==expected,'El paquete de configuración no coincide con el SHA256 fijado.'
PY
# El paquete oficial administra la clave y la configuración del repositorio ROS.
dpkg -i "$TASK_REPO_DEB"
apt-get update
# --no-remove detiene APT si la resolución pretendiera eliminar paquetes.
DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends --no-remove install \
  ros-humble-ros-base ros-humble-rviz2 ros-humble-demo-nodes-cpp \
  ros-humble-demo-nodes-py python3-colcon-common-extensions ros-humble-ackermann-msgs \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro python3-transforms3d python3-venv ffmpeg
DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends --no-remove install \
  git python3-dev python3-pip build-essential python3-numpy python3-pil \
  python3-yaml python3-pytest python3-future xvfb libx11-6 libxtst6 xauth libgl1 libglu1-mesa \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rclpy ros-humble-teleop-twist-keyboard
test -r /opt/ros/humble/setup.bash
dpkg-query -W ros-humble-ros-base ros-humble-rclpy ros-humble-rviz2 \
  ros-humble-ackermann-msgs ros-humble-nav2-map-server python3-colcon-common-extensions \
  > "$TASK_LOG_DIR/native_packages.txt"
printf 'installed\n' > "$TASK_LOG_DIR/state"
printf '\nROS 2 Humble instalado mediante APT en /opt/ros/humble.\n'
printf 'Activación: source /opt/ros/humble/setup.bash\n'
printf 'Siguiente paso, SIN sudo: bash installation/setup_workspace.sh\n'
