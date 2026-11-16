# Fuentes y atribuciones

## Código propio

`daniel_gap/`, las herramientas de medición, preparación y auditoría y la documentación del proyecto se distribuyen bajo la [licencia MIT](LICENSE) de Daniel Bejarano. Los archivos externos conservan sus respectivos avisos de autoría.

## Motor F1TENTH Gym

- Fuente: https://github.com/f1tenth/f1tenth_gym
- Commit fijado: `4fdb9c7e6fb7c701290f4dc18377d07c1681724f`.
- Distribución Python: `f110-gym` 0.2.1; API Gym 0.19.0.
- Se descarga como dependencia a `external/f1tenth_gym/`, excluida de Git. Se conserva su licencia original y no se atribuye ese motor al autor de la actividad.
- [simulator-source.json](installation/simulator-source.json) incluye hashes del código físico y del simulador utilizados para verificar la fuente. El checkout original de las mediciones coincide con ese commit.

## Bridge ROS 2

El paquete incluido en `ros2_validation/simulator_bridge/` deriva del bridge de Hongrui Zheng/Billy Zheng (avisos MIT de 2020 conservados en los archivos). La base utilizada en el curso se obtuvo de https://github.com/widegonz/F1Tenth-Repository, revisión `7e79e592232571e2a340287e7e2a8f22a0812927`.

Las modificaciones de la actividad incluyen reloj simulado, bandera de contacto del motor, instrumentación de la ejecución individual, modo lockstep y resolución portable del mapa. Se conserva la autoría y los metadatos del paquete externo.

## Mapas y herramientas

Los pares PNG/YAML del ZIP `Mapas-F1Tenth-v2.zip` son recursos del curso. No se ha identificado una licencia de redistribución en ese ZIP. Se excluyen de la licencia MIT del proyecto y del Git normal; cada participante los importa desde el material docente con verificación de SHA256. No existe aún una URL pública de descarga confirmada.

ROS 2, RViz, Nav2, colcon, FFmpeg, Xvfb y las dependencias Python mantienen sus propias licencias. El instalador descarga el paquete oficial de configuración APT desde https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0 y comprueba el hash fijado antes de instalarlo.
