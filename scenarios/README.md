# Parte 2: obstáculos estáticos y adelantamiento

Los dos escenarios se ejecutan por separado sobre Sakhir. El primero utiliza una copia del mapa con cinco obstáculos ocupados; el segundo utiliza el mapa original y dos vehículos físicos del simulador. No se cambia el modelo del coche, el paso físico de 0,01 s ni el mapa del curso. LiDAR decide dirección y velocidad; odometría mide vueltas, sectores y adelantamientos.

La tanda nativa del 22 de noviembre completó ambas pruebas: diez vueltas estáticas y 13/10 vueltas dinámicas, con cuatro adelantamientos y sin contactos registrados. Consulta los [tiempos por vuelta y los informes de auditoría](RESULTADOS.md).

## Preparación

Completa la instalación del [README principal](../README.md), importa el ZIP docente y compila el workspace. Después de cambiar código, comprueba el entorno y las pruebas:

```bash
bash installation/setup_workspace.sh
./start_ros2.sh check
./start_ros2.sh exec python3 -m pytest -q tests
```

Deben aprobar las 19 pruebas actuales. Usa Ubuntu 22.04 amd64 y ROS 2 Humble. El bootstrap instala las dependencias dentro del propio clon; no se necesitan el entorno, los mapas derivados ni los rosbags del autor.

## 1. Cinco obstáculos estáticos

```bash
./start_ros2.sh exec python3 scenarios/build_obstacle_map.py
```

Genera `results/Sakhir_obstacles_5.png`, `.yaml` y un informe `.json`. El original no se sobrescribe. Si esa salida ya existe, consérvala; para otra generación usa `--out results/Sakhir_obstacles_repeticion`. La disposición fija está en [static_obstacles.yaml](static_obstacles.yaml): cinco discos de 0,70 m de diámetro, separados a lo largo de la pista. El generador comprueba exactamente cinco componentes nuevas dentro del carril y un paso local para un círculo de radio 0,35 m. Esa comprobación geométrica por sí sola no acredita vueltas.

Para abrir el mapa en el escritorio con el coche detenido:

```bash
ROS_DOMAIN_ID=187 LIBGL_ALWAYS_SOFTWARE=1 ./start_ros2.sh ros2 launch scenarios/obstacles_map.launch.py
```

Los cilindros rojos y las etiquetas O1–O5 identifican las mismas posiciones que ya están ocupadas en el PNG; la detección y colisión proceden del mapa, no de esas etiquetas. Ctrl+C en esa terminal cierra la vista de inspección. Ciérrala antes de validar para reducir carga del equipo.

### Conducir, contar diez vueltas y grabar RViz real

El perfil `obstacles` conserva Follow the Gap, la inflación de 0,40 m y la ley reactiva de velocidad. Selecciona el centro del hueco libre para evitar las inversiones de marcha observadas con el objetivo más lejano. El perfil original `fast` sigue disponible y se utiliza en la prueba dinámica y los resultados de la primera parte. [static_controller.yaml](static_controller.yaml) registra la configuración de 6 m/s, ganancia 0,7, parámetros lateral/frenado 6 y peso de dirección nueva 0,8.

Desde la raíz del clon, elige una carpeta, dominio DDS y pantalla virtual libres. El archivo de inicio tampoco debe existir; si el `test` falla, cambia su nombre antes de ejecutar el runner:

```bash
test ! -e .runtime/iniciar_estatica_01 &&
./start_ros2.sh exec python3 scenarios/run_static.py \
  --out ros2_validation/mis_pruebas/parte2_estatica_01 \
  --mode lockstep --max-speed 6 --controller-profile obstacles \
  --gain .7 --lateral-accel 6 --brake-accel 6 --smooth 11 --alpha .8 \
  --map-stem results/Sakhir_obstacles_5 --display :189 --domain 189 \
  --record-video --start-file .runtime/iniciar_estatica_01 --timeout-wall 1800
```

Espera `READY_FOR_CONTROLLER`. En otra terminal, desde la misma raíz:

```bash
xdg-open ros2_validation/mis_pruebas/parte2_estatica_01/preflight.png
touch .runtime/iniciar_estatica_01
```

Tras revisar la disposición, el archivo de inicio autoriza empezar la conducción. No es una imagen animada: dos procesos reales de RViz observan la misma instancia de F1TENTH y FFmpeg captura X11 continuamente. El coche puede no aparecer en la imagen previa con reloj simulado cero; debe verse al avanzar la ejecución. El runner termina después de diez vueltas y la observación del frenado. Usa otro nombre de carpeta y de archivo de inicio en cada repetición.

Audita el bag cerrado:

```bash
./start_ros2.sh exec python3 scenarios/validate_static.py \
  --run ros2_validation/mis_pruebas/parte2_estatica_01/run \
  --max-speed 6 --map-yaml results/Sakhir_obstacles_5.yaml \
  --output ros2_validation/mis_pruebas/parte2_estatica_01/checks_static.json
```

La tanda debe indicar `completed: true`, diez tiempos y ningún contacto registrado. La auditoría debe terminar con código 0 y `passed: true`, con sus **19 controles aprobados**. Añade a los controles de la primera parte la coincidencia del mapa auditado con el grabado y el paso O1→O2→O3→O4→O5 en cada vuelta. Rechaza los recorridos que invierten la marcha y cruzan la meta sin recorrer el circuito.

## 2. Segundo robot a cinco metros y adelantamiento

[dynamic_controllers.yaml](dynamic_controllers.yaml) inicia el ego en `(0,5;15)` m y el segundo en `(0,5;10)` m, ambos orientados hacia −π/2: son exactamente cinco metros de adelanto. Ambos utilizan instancias independientes del `FastGapController` de la primera parte, con máximos de 5 y 3,5 m/s y parámetros lateral/frenado de 3. El LiDAR incluye al otro vehículo. Las poses se usan exclusivamente para registrar la separación, vueltas y adelantamientos.

```bash
./start_ros2.sh exec python3 scenarios/run_dynamic.py \
  --out ros2_validation/mis_pruebas/parte2_dinamica_01 \
  --mode lockstep --record-video
```

El runner usa por defecto la pantalla `:190` y el dominio `190`, que deben estar libres; consulta `--help` para cambiarlos o aumentar el timeout. Inicia el bridge, dos controladores, dos RViz, monitor, rosbag y captura de video. Cierra el lanzamiento con Ctrl+C si necesitas interrumpirlo; conserva el diagnóstico, pero una tanda interrumpida no acredita el objetivo.

Cada vehículo sigue circulando hasta que **ambos han completado al menos diez vueltas**. Después ambos frenan conservando dirección reactiva y se observa su parada. El más rápido puede completar vueltas adicionales: se cuentan todas y se muestran explícitamente; las primeras diez se separan para comparar tiempos. Así no se convierte artificialmente en un obstáculo aparcado antes de que termine el segundo coche.

El bridge espera órdenes de ambos controladores en cada paso lockstep y publica un flag de colisión independiente por vehículo. No aumenta el tiempo simulado sin un paso físico. La sincronización y los flags son instrumentación; no alteran la física.

El runner audita automáticamente al cerrar. Para repetir manualmente la auditoría en un archivo nuevo:

```bash
./start_ros2.sh exec python3 scenarios/validate_dynamic.py \
  --run ros2_validation/mis_pruebas/parte2_dinamica_01/run \
  --config ros2_validation/mis_pruebas/parte2_dinamica_01/dynamic_controllers.yaml \
  --map-yaml results/Sakhir_map.yaml \
  --output ros2_validation/mis_pruebas/parte2_dinamica_01/checks_dynamic_repeticion.json
```

Debe indicar `passed: true` y todos los controles aprobados: al menos diez vueltas por robot, ninguno de los flags registrados positivo, centros de odometría dentro del carril, límites físicos de órdenes, parada final, salida a cinco metros y adelantamiento. El adelantamiento exige observar al ego cerca y por detrás, luego por delante, con rumbos compatibles y persistencia de 0,5 s; no basta una diferencia de vueltas. El auditor también comprueba cinco sectores ordenados del circuito en cada vuelta real de ambos coches; son referencias de medición, no obstáculos en esta segunda prueba.

## Tiempos, videos y alcance de evidencia

`lockstep` puede avanzar más rápido que el reloj real. Los tiempos de vuelta proceden de `/clock`; la duración del video corresponde al tiempo de pared. Para usar temporizadores normales cambia `--mode lockstep` por `--mode wall` y permite un timeout mayor si lo necesitas. Una captura lockstep sigue siendo una grabación directa del simulador real; no se acelera ni reconstruye a partir de CSV.

Los videos se generan en la carpeta de cada tanda. Los MP4 y rosbags no forman parte del Git normal. Las grabaciones del 26 de noviembre de 2026, con consola visible y una vista de seguimiento más amplia, están en YouTube: [cinco obstáculos y diez vueltas](https://youtu.be/4yqzK4zvlZ8) y [dos robots y adelantamiento](https://youtu.be/xx3N9OuGNBI). El enlace de YouTube de la primera parte acredita aquella tanda.

Para reproducir un video y comprobar su decodificación completa:

```bash
ffplay -autoexit ruta/de/la/tanda/evidencia_ros2_rviz.mp4
ffmpeg -v error -i ruta/de/la/tanda/evidencia_ros2_rviz.mp4 -f null -
```

El video dinámico se llama `evidencia_dynamic_ros2_rviz.mp4`. Usa su nombre real al reproducirlo. Si falla la aceleración gráfica del reproductor, utiliza decodificación por software. Las copias de compatibilidad conservan todos los fotogramas y la duración y añaden una pista AAC silenciosa.

El observador conserva los mensajes recibidos; no se presume capturar cada publicación. El reconteo comparte reloj y algoritmo con el controlador, y la prueba geométrica evalúa centros de odometría. Las banderas de colisión provienen del motor. Los nuevos resultados no garantizan un puesto ni una nota: faltan la competencia comparada y la sustentación docente.
