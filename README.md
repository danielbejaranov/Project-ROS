# F1TENTH Sakhir — Follow the Gap

**Autor:** Daniel Bejarano — Bejarano Villamar Daniel Emanuel.

Proyecto de navegación reactiva para F1TENTH en Sakhir. El entorno se prepara dentro del propio clon, con Python 3.10 y versiones fijadas del simulador.

## Preparar el entorno

Se requiere Ubuntu 22.04 amd64, Internet, permisos sudo y el archivo `Mapas-F1Tenth-v2.zip` del curso. Los mapas no se redistribuyen en el repositorio. Utiliza rutas sin espacios para el clon y el workspace.

```bash
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/danielbejaranov/Project-ROS.git
cd Project-ROS
sudo bash installation/install_ros2_humble.sh
python3 installation/import_sakhir_map.py --zip "$HOME/Downloads/Mapas-F1Tenth-v2.zip"
bash installation/setup_workspace.sh --dependencies-only
```

El instalador prepara ROS 2 Humble y las herramientas del sistema. El bootstrap crea `.venv/` con acceso a ROS del sistema y descarga el motor en `external/f1tenth_gym/`, en el commit de [simulator-source.json](installation/simulator-source.json). No utiliza un entorno del equipo del autor ni cambia `.bashrc`.

## Controlador y pruebas

`daniel_gap/core.py` contiene Follow the Gap y `LapCounter`; `fast_core.py` aplica el perfil rápido, y `ros_node.py` expone el nodo ROS. Los parámetros están en `config/fast_controller.yaml`. La dirección y la velocidad dependen del LiDAR; la odometría se reserva para contar y cronometrar vueltas. No se usan waypoints para conducir.

Después de preparar las dependencias anteriores, ejecuta desde la raíz:

```bash
.venv/bin/python -m pytest -q tests
.venv/bin/python optimization_20261115/benchmark_fast.py \
  --name prueba_rapida_01 --vmax 20 --lat 12.5 --brake 8.25 \
  --ros-float32 --settle-seconds 4
```

Las cinco pruebas verifican escaneos inválidos, corredor libre y el conteo de vueltas. El ensayo directo utiliza el mapa importado y guarda sus propios resultados en una carpeta nueva de `optimization_20261115/gym_runs/`. Un ensayo válido termina con diez vueltas y sin colisiones; no sustituye una ejecución posterior en ROS 2 y RViz.

El contador detecta cruces de meta en el sentido correcto, exige distancia y tiempo mínimos y rechaza saltos, reloj invertido y contactos. La conducción continúa al cruzar la meta sin reiniciar el coche; al terminar se mantiene la dirección reactiva durante el frenado.


Consulta [fuentes y atribuciones](THIRD_PARTY.md). El código propio se distribuye bajo [MIT](LICENSE); los mapas y las dependencias mantienen sus licencias.
