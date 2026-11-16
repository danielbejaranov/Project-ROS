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

Este estado prepara las dependencias; la compilación de los paquetes propios se incorpora junto con el controlador y la integración ROS 2.

Consulta [fuentes y atribuciones](THIRD_PARTY.md). El código propio se distribuye bajo [MIT](LICENSE); los mapas y las dependencias mantienen sus licencias.
