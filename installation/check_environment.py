#!/usr/bin/env python3
"""Comprueba los requisitos de réplica sin iniciar nodos, DDS ni simulación.

Ejecutar mediante ./start_ros2.sh check para activar ROS, el entorno Python y
el overlay. Cada comprobación imprime PASS o FAIL; el código de salida es 1
si falta algún requisito. --output guarda un informe JSON nuevo opcional.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import sys

from import_sakhir_map import EXPECTED_SHA256


PROJECT = Path(__file__).resolve().parents[1]


def contained(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--requirements", type=Path, default=PROJECT / "installation/requirements-sim.txt",
                        help="requisitos Python con versiones fijadas")
    parser.add_argument("--overlay", type=Path,
                        default=Path(os.environ.get("DANIEL_ROS2_WS", os.environ.get("ROS2_WS", str(PROJECT / ".ros2_ws")))),
                        help="workspace ROS 2 (por defecto DANIEL_ROS2_WS o .ros2_ws)")
    parser.add_argument("--map", type=Path, dest="map_stem",
                        default=Path(os.environ.get("DANIEL_F1TENTH_MAP", str(PROJECT / "results/Sakhir_map"))),
                        help="ruta sin extensión del mapa Sakhir (o su YAML)")
    parser.add_argument("--output", type=Path, help="informe JSON nuevo; no reemplaza archivos existentes")
    args = parser.parse_args()
    if args.output is not None:
        if args.output.exists() or args.output.is_symlink():
            parser.error(f"El informe ya existe; elija otra ruta: {args.output}")
        if not args.output.parent.is_dir():
            parser.error(f"No existe el directorio del informe: {args.output.parent}")

    checks: list[dict] = []

    def record(name: str, ok: bool, detail: str, hint: str = "") -> None:
        checks.append({"name": name, "status": "PASS" if ok else "FAIL", "detail": detail,
                       **({"hint": hint} if not ok and hint else {})})
        print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")
        if not ok and hint:
            print(f"     Acción: {hint}")

    def attempt(name: str, operation, hint: str = "") -> None:
        try:
            detail = operation()
            record(name, True, str(detail))
        except Exception as error:
            record(name, False, f"{type(error).__name__}: {error}", hint)

    def os_release() -> str:
        release = platform.freedesktop_os_release()
        if release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "22.04":
            raise ValueError(f"se requiere Ubuntu 22.04; detectado {release.get('PRETTY_NAME', release)}")
        return release["PRETTY_NAME"]

    attempt("sistema", os_release, "Use Ubuntu 22.04 LTS de 64 bits, según README.")
    record("Python", sys.version_info[:2] == (3, 10),
           f"{platform.python_version()} — {sys.executable}", "Ejecute ./start_ros2.sh check con Python 3.10.")
    record("arquitectura", platform.machine() == "x86_64", platform.machine(),
           "La plataforma validada es Ubuntu 22.04 amd64; otras arquitecturas necesitan validación adicional.")
    ros_setup = Path("/opt/ros/humble/setup.bash")
    record("ROS 2 Humble instalado", ros_setup.is_file(), str(ros_setup),
           "Complete la instalación de ROS 2 Humble del README.")
    record("ROS_DISTRO", os.environ.get("ROS_DISTRO") == "humble", repr(os.environ.get("ROS_DISTRO")),
           "Ejecute ./start_ros2.sh check para cargar /opt/ros/humble/setup.bash.")
    overlay = args.overlay.expanduser().resolve()
    record("overlay construido", (overlay / "install/setup.bash").is_file(), str(overlay / "install/setup.bash"),
           "Construya los dos paquetes con installation/setup_workspace.sh.")

    def versions() -> str:
        pinned = []
        problems = []
        for number, line in enumerate(args.requirements.read_text(encoding="utf-8").splitlines(), 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            line = re.split(r"\s+#", line, maxsplit=1)[0].strip()
            match = re.fullmatch(r"([A-Za-z0-9_.-]+)(?:\[[^]]+\])?==([^\s;]+)", line)
            if not match:
                problems.append(f"línea {number} sin versión exacta compatible: {line}")
                continue
            name, expected = match.groups()
            try:
                actual = importlib.metadata.version(name)
                pinned.append(f"{name}=={actual}")
                if actual != expected:
                    problems.append(f"{name}: instalado {actual}, requerido {expected}")
            except importlib.metadata.PackageNotFoundError:
                problems.append(f"{name}=={expected}: no instalado")
        if not pinned and not problems:
            problems.append("el archivo no contiene dependencias fijadas")
        if problems:
            raise ValueError("; ".join(problems))
        return ", ".join(pinned)

    attempt("versiones Python fijadas", versions,
            "Instale installation/requirements-sim.txt en el entorno Python del proyecto.")
    def gym_version() -> str:
        actual = importlib.metadata.version("gym")
        if actual != "0.19.0":
            raise ValueError(f"Gym {actual}; se requiere 0.19.0")
        return actual
    attempt("versión Gym", gym_version, "Complete installation/setup_workspace.sh; Gym se instala por separado.")
    for module in ("numpy", "scipy", "numba", "PIL", "yaml", "gym", "f110_gym", "transforms3d",
                   "rclpy", "rosbag2_py", "sensor_msgs.msg", "nav_msgs.msg", "ackermann_msgs.msg",
                   "std_msgs.msg", "rosgraph_msgs.msg", "tf2_ros", "launch", "launch_ros"):
        def check_import(name=module) -> str:
            imported = importlib.import_module(name)
            origin = str(getattr(imported, "__file__", "namespace importado"))
            if name == "rclpy" and not contained(Path(origin), Path("/opt/ros/humble")):
                raise ValueError(f"rclpy no procede de /opt/ros/humble: {origin}")
            if name == "f110_gym" and not contained(Path(origin), PROJECT / "external/f1tenth_gym"):
                raise ValueError(f"se importó una copia externa al clon: {origin}")
            return origin
        attempt(f"import {module}", check_import, "Revise la instalación y ejecute ./start_ros2.sh check.")

    def simulator_source() -> str:
        spec = json.loads((PROJECT / "installation/simulator-source.json").read_text())
        checkout = PROJECT / "external/f1tenth_gym"
        for name, expected in spec["sha256"].items():
            path = checkout / name
            if not contained(path, checkout):
                raise ValueError(f"ruta fuera del simulador: {name}")
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected:
                raise ValueError(f"{name}: SHA-256 {actual}; esperado {expected}")
        version = importlib.metadata.version(spec["python_distribution"])
        if version != spec["version"]:
            raise ValueError(f"versión del motor {version}; se requiere {spec['version']}")
        return f"{len(spec['sha256'])} archivos coinciden con el commit fijado {spec['commit']}; versión {version}"
    attempt("fuente del motor F1TENTH", simulator_source,
            "Complete installation/setup_workspace.sh sin reutilizar otra instalación de f110_gym.")

    # Import only module definitions: never instantiate nodes or call rclpy.init.
    sys.path.insert(0, str(PROJECT / "ros2_validation/simulator_bridge"))
    sys.path.insert(0, str(PROJECT))
    for name, expected_dir in (("daniel_gap.ros_node", PROJECT / "daniel_gap"),
                               ("f1tenth_gym_ros.gym_bridge", PROJECT / "ros2_validation/simulator_bridge")):
        def source_import(module=name, directory=expected_dir) -> str:
            imported = importlib.import_module(module)
            origin = Path(imported.__file__).resolve()
            if not contained(origin, directory):
                raise ValueError(f"se importó otra copia: {origin}; se esperaba {directory}")
            return str(origin)
        attempt(f"fuente {name}", source_import, "Use el clon actual y reconstruya el overlay si procede.")

    for package in ("daniel_gap", "f1tenth_gym_ros", "rviz2", "nav2_map_server", "nav2_lifecycle_manager",
                    "robot_state_publisher", "joint_state_publisher", "xacro"):
        def package_path(name=package) -> str:
            from ament_index_python.packages import get_package_prefix
            prefix = Path(get_package_prefix(name)).resolve()
            if name in ("daniel_gap", "f1tenth_gym_ros") and not contained(prefix, overlay / "install"):
                raise ValueError(f"otro overlay está activo: {prefix}; esperado {overlay / 'install'}")
            return str(prefix)
        attempt(f"paquete ROS {package}", package_path, "Instale las dependencias y cargue el overlay indicado en README.")

    map_stem = args.map_stem.expanduser()
    if map_stem.suffix in (".yaml", ".png"):
        map_stem = map_stem.with_suffix("")
    for extension in (".png", ".yaml"):
        map_path = Path(str(map_stem) + extension)
        expected_name = "Sakhir_map" + extension
        def check_map(path=map_path, name=expected_name) -> str:
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != EXPECTED_SHA256[name]:
                raise ValueError(f"{path}: SHA-256 {actual}; esperado {EXPECTED_SHA256[name]}")
            return f"{path.resolve()} — {actual}"
        attempt(f"mapa {expected_name}", check_map,
                "Importe el ZIP original con installation/import_sakhir_map.py --zip RUTA_AL_ZIP.")

    def configuration() -> str:
        import yaml
        config = PROJECT / "config/fast_controller.yaml"
        params = yaml.safe_load(config.read_text())["daniel_gap"]["ros__parameters"]
        required = {"controller_profile": "fast", "target_laps": 10, "max_speed": 20.0,
                    "gain": 0.7, "lateral_accel": 12.5, "brake_accel": 8.25, "smooth": 31, "alpha": 0.6}
        differences = [f"{key}={params.get(key)!r} (esperado {value!r})"
                       for key, value in required.items() if params.get(key) != value]
        if differences:
            raise ValueError("; ".join(differences))
        map_yaml = Path(str(map_stem) + ".yaml")
        image = yaml.safe_load(map_yaml.read_text())["image"]
        if image != "Sakhir_map.png":
            raise ValueError("el YAML del mapa no referencia image: Sakhir_map.png")
        return f"{config}: perfil final de 10 vueltas; YAML del mapa relativo"
    attempt("configuración reproducible", configuration, "Use config/fast_controller.yaml y el mapa original sin modificar.")

    for executable in ("ros2", "Xvfb", "ffmpeg", "ffprobe", "colcon"):
        resolved = shutil.which(executable)
        record(f"herramienta {executable}", resolved is not None, resolved or "no encontrada en PATH",
               "Instale las herramientas indicadas en README y active el entorno del proyecto.")

    failed = sum(check["status"] == "FAIL" for check in checks)
    report = {"status": "FAIL" if failed else "PASS", "project": str(PROJECT),
              "python": sys.executable, "overlay": str(overlay), "checks": checks,
              "passed": len(checks) - failed, "failed": failed,
              "note": "Comprobación estática del entorno; no se inició DDS ni una simulación."}
    if args.output is not None:
        try:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
        except OSError as error:
            print(f"FAIL escritura de informe: {error}", file=sys.stderr)
            return 1
    print(f"\n{report['status']}: {report['passed']}/{len(checks)} comprobaciones correctas; {failed} fallos.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
