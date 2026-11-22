# F1TENTH Sakhir — Follow the Gap con ROS 2

**Autor:** Daniel Bejarano — Bejarano Villamar Daniel Emanuel.

Este proyecto conduce un F1TENTH simulado mediante LiDAR, cuenta diez vueltas consecutivas y registra sus tiempos y contactos. **F1TENTH Gym es el simulador; RViz visualiza su estado.** La odometría se utiliza para medir vueltas, no para seguir una trayectoria. No se emplean waypoints ni un mapa para decidir la dirección o la velocidad.

[Ver el video de la ejecución real en YouTube](https://youtu.be/oMR08ioytbE).

La grabación muestra diez vueltas en Sakhir, el contador y el cronómetro, con vista general y cercana de RViz. La tanda registró cero contactos y aprobó 17/17 comprobaciones de auditoría. Mejor vuelta: **50,82 s**; media: **51,340 s**. El video se entrega mediante ese enlace; los archivos MP4 no forman parte del repositorio.

Esta guía prepara el proyecto dentro de su propio clon. No requiere las carpetas, los rosbags ni el entorno Python del equipo del autor. Ejecuta los comandos desde la raíz del repositorio, salvo cuando se indique otra cosa.

La segunda parte tiene [dos escenarios reproducibles: cinco obstáculos y adelantamiento a otro robot](scenarios/README.md). Se ejecutan y auditan por separado. Los resultados y el video de esta guía principal corresponden a la primera parte, sin esos obstáculos ni segundo robot.

## 1. Requisitos y alcance

- Ubuntu **22.04 amd64/x86_64**, Python **3.10** del sistema y ROS 2 **Humble** instalado por APT en `/opt/ros/humble`. Es la plataforma probada. No se ha validado esta guía en Windows, macOS, ARM ni otras versiones de Ubuntu/ROS.
- Acceso a Internet para APT, PyPI y GitHub, y permisos `sudo` para instalar dependencias del sistema. La compilación y las ejecuciones se hacen sin `sudo`.
- Guarda el clon y su workspace en rutas **sin espacios**, por ejemplo `~/f1tenth-sakhir`. Los comandos de xacro del bridge requieren esa estructura. La ruta del ZIP del mapa sí puede contener espacios.
- Una sesión gráfica para la demostración interactiva. La validación automatizada utiliza Xvfb y puede grabar RViz sin una pantalla física.
- Espacio para las dependencias y los nuevos registros: reservar al menos **5 GB libres** como margen de trabajo; el consumo depende del número de tandas conservadas. Cada validación genera un rosbag grande.
- El archivo **`Mapas-F1Tenth-v2.zip` suministrado por el curso**. Debe contener `Sakhir_map.png` y `Sakhir_map.yaml`. No se ha identificado una URL pública ni permiso de redistribución de esos mapas; se obtienen del material docente. Si no tienes el ZIP, solicítalo al docente antes de continuar. La licencia MIT del código no incluye estos recursos.

Las versiones del motor y las dependencias Python se fijan en [simulator-source.json](installation/simulator-source.json) y [requirements-sim.txt](installation/requirements-sim.txt). Se utiliza Gym 0.19; Gymnasium tiene otra API y no lo sustituye en este proyecto.

## 2. Obtener el código

El repositorio es público: no necesitas una cuenta de GitHub ni autenticación para clonarlo. Instala Git y descarga el proyecto:

```bash
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/danielbejaranov/Project-ROS.git
cd Project-ROS
```

También puedes usar **Code → Download ZIP**, extraerlo y abrir una terminal en la carpeta que contiene este `README.md`. Los pasos restantes son iguales.

## 3. Instalar ROS 2 y las herramientas del sistema

```bash
sudo bash installation/install_ros2_humble.sh
```

El instalador admite Ubuntu 22.04 amd64. Descarga una versión fija del paquete oficial `ros2-apt-source`, verifica su SHA256 e instala ROS, RViz, colcon, mensajes Ackermann, Nav2 Map Server, rosbag2, Xvfb, FFmpeg y las herramientas de compilación/Python. Si ROS ya está instalado, ejecútalo igualmente para completar esas dependencias. No modifica `.bashrc`. Revisa cualquier error de APT antes de continuar; una instalación fallida no equivale a un entorno preparado.

La referencia del procedimiento es la [instalación oficial de ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). El script del proyecto fija sus entradas para evitar depender de un archivo `.deb` privado del autor.

## 4. Importar y comprobar el mapa

Indica la ubicación real del ZIP; se admiten espacios en la ruta si mantienes las comillas:

```bash
python3 installation/import_sakhir_map.py \
  --zip "$HOME/Downloads/Mapas-F1Tenth-v2.zip"
```

El importador copia únicamente el par de Sakhir a `results/`, comprueba sus hashes contra el mapa utilizado en las mediciones y conserva los archivos originales. Si detecta otro mapa o archivos locales diferentes, se detiene en lugar de reemplazarlos. El ZIP no necesita permanecer dentro del repositorio.

## 5. Preparar Python y compilar los dos paquetes

Abre una terminal nueva, sin Conda ni otro entorno virtual activo, sitúate en la raíz del repositorio y ejecuta:

```bash
bash installation/setup_workspace.sh
./start_ros2.sh check
./start_ros2.sh exec python3 -m pytest -q tests
```

La preparación crea los siguientes directorios locales, excluidos de Git:

```text
.venv/                     Python del proyecto con acceso a paquetes ROS del sistema
external/f1tenth_gym/       Motor externo en el commit fijado
.ros2_ws/src/daniel_gap     Enlace al paquete del controlador
.ros2_ws/src/f1tenth_gym_ros Enlace al bridge incluido en este repositorio
.ros2_ws/install/           Resultado de colcon
.runtime/                  Cachés y logs de las nuevas ejecuciones
```

Los dos paquetes se compilan como hermanos. Clonar únicamente este repositorio dentro de un `src/` y ejecutar colcon sin preparar el bridge anidado no es el procedimiento de esta guía.

`check` debe terminar con código **0** y todas sus comprobaciones aprobadas: Python y módulos, ROS nativo, ambos paquetes, herramientas y mapa. Las pruebas actuales deben mostrar **13 passed** (las cinco del núcleo y las regresiones de la segunda parte). No continúes si falla una comprobación. La primera preparación y la primera ejecución pueden tardar por las descargas y la compilación JIT de Numba.

`start_ros2.sh` activa ROS, `.venv` y el overlay en cada llamada. No necesitas activar nada manualmente ni añadir rutas al perfil de tu usuario. En cada terminal nueva, vuelve a la raíz y utiliza el mismo lanzador.

## 6. Abrir una demostración en RViz

```bash
./start_ros2.sh demo
```

Debe abrirse RViz con el mapa de Sakhir, el vehículo y el LiDAR. El controlador empieza a conducir con el perfil rápido de [config/fast_controller.yaml](config/fast_controller.yaml). Cada inicio crea un CSV de vueltas en `ros2_validation/demo/<fecha UTC>/laps.csv`. Esa demostración no graba por sí sola una auditoría completa ni un video.

En otra terminal, desde la raíz del mismo clon:

```bash
./start_ros2.sh status
```

Se muestra el estado, el número de vueltas y sus tiempos. Al completar la décima vuelta se solicita detener el coche; el simulador y RViz siguen abiertos. **Ctrl+C en la terminal de la demo** cierra su lanzamiento; Ctrl+C en `status` cierra únicamente esa vista. Cierra la demo antes de realizar la validación siguiente para reducir carga del equipo.

## 7. Ejecutar diez vueltas, grabar y auditar

La siguiente ejecución usa temporizadores normales. Elige una carpeta de salida **nueva**; el runner rechaza sobrescribir una existente. El dominio DDS `95` y la pantalla virtual `:111` deben estar libres:

```bash
./start_ros2.sh exec python3 ros2_validation/run_candidate.py \
  --out ros2_validation/mis_pruebas/tanda_wall_01 \
  --mode wall --max-speed 20 --controller-profile fast \
  --gain .7 --lateral-accel 12.5 --brake-accel 8.25 \
  --smooth 31 --alpha .6 --display :111 --domain 95 --record-video
```

El proceso inicia Xvfb, RViz, simulador, observador, controlador y grabación. RViz se captura en la pantalla virtual y **no aparece como ventana en tu escritorio**. Imprime avances cada 30 segundos y los tiempos al completar vueltas. Finaliza automáticamente tras diez vueltas y un intervalo adicional de frenado. Puede tardar alrededor de nueve minutos en este modo, más el arranque; depende de la capacidad del equipo. El timeout de pared predeterminado es de 1200 segundos. Ctrl+C interrumpe y conserva lo registrado, pero una tanda interrumpida no se considera válida.

Después, ejecuta la auditoría sobre esa misma carpeta:

```bash
./start_ros2.sh exec python3 ros2_validation/validate_run.py \
  --run ros2_validation/mis_pruebas/tanda_wall_01/run \
  --max-speed 20 \
  --output ros2_validation/mis_pruebas/tanda_wall_01/checks_ros.json
```

Una tanda aceptada debe tener `completed: true`, diez tiempos y `collision: false` en `run/summary_ros.json`. Además, `checks_ros.json` debe indicar **`passed: true` y los 17 controles aprobados**, con salida de auditoría 0. La auditoría revisa límites de órdenes, mensajes de contacto, posiciones registradas, reconteo de vueltas, reloj y parada posterior. Una colisión, menos de diez vueltas, falta de evidencia o frenado inválido impiden dar la tanda por válida.

Para un ensayo **lockstep**, repite ambos comandos cambiando la carpeta por `tanda_lockstep_01` y `--mode wall` por `--mode lockstep`. Este modo espera una orden por paso de simulación y puede avanzar más rápido que el reloj real. Sus resultados se comparan por separado. Los tiempos de vuelta siempre proceden del reloj **simulado**, no de la duración del video o del proceso.

Archivos de una nueva validación:

| Archivo | Uso |
|---|---|
| `run/laps_ros.csv` | Tiempos de las diez vueltas |
| `run/summary_ros.json` | Finalización, colisiones y recepción de mensajes |
| `run/rosbag/` | Grabación para volver a auditar esa tanda |
| `checks_ros.json` | Resultado de los controles de auditoría |
| `manifest.json` | Parámetros, rutas, hashes y procesos de la ejecución |
| `evidencia_ros2_rviz.mp4` | Video de RViz si se usó `--record-video` |
| `logs/` | Diagnóstico de cada proceso |

Para reproducir tu video:

```bash
ffplay -autoexit ros2_validation/mis_pruebas/tanda_wall_01/evidencia_ros2_rviz.mp4
```

FFmpeg genera H.264, píxeles `yuv420p` y MP4 con `faststart`. Para verificar que el archivo se decodifica completo:

```bash
ffmpeg -v error \
  -i ros2_validation/mis_pruebas/tanda_wall_01/evidencia_ros2_rviz.mp4 \
  -f null -
```

Sin mensajes de error y con salida 0, el archivo se ha decodificado correctamente.

### Grabar con las dos vistas del video de evidencia

El comando anterior graba una vista de RViz a 1280×900. Para usar la disposición de **1920×1080 con pista completa y vehículo cercano** de la grabación publicada, utiliza el runner de presentación siguiente. Comparte controlador, física, contador y monitor con la validación anterior.

Desde la raíz, en la primera terminal, elige otra carpeta, pantalla y dominio libres. El archivo de inicio tampoco debe existir:

```bash
test ! -e .runtime/iniciar_doble_01 &&
./start_ros2.sh exec python3 ros2_validation/run_real_capture.py \
  --out ros2_validation/mis_pruebas/tanda_doble_01 \
  --mode wall --max-speed 20 --controller-profile fast \
  --gain .7 --lateral-accel 12.5 --brake-accel 8.25 \
  --smooth 31 --alpha .6 --display :112 --domain 96 --record-video \
  --start-file .runtime/iniciar_doble_01
```

Espera el mensaje `READY_FOR_CONTROLLER`. En una segunda terminal situada en la raíz del mismo clon, abre la imagen previa, comprueba que ambas vistas y el contador se ven y crea el archivo que inicia la conducción:

```bash
xdg-open ros2_validation/mis_pruebas/tanda_doble_01/preflight.png
touch .runtime/iniciar_doble_01
```

La imagen previa es estática; el MP4 empieza a grabarse justo antes de iniciar el controlador. Espera a que la primera terminal termine y audita su propia salida:

```bash
./start_ros2.sh exec python3 ros2_validation/validate_run.py \
  --run ros2_validation/mis_pruebas/tanda_doble_01/run \
  --max-speed 20 \
  --output ros2_validation/mis_pruebas/tanda_doble_01/checks_ros.json
```

También debe indicar `passed: true` y 17/17. Conserva el archivo de inicio con esa tanda y usa otro nombre al repetir. Las dos ventanas pertenecen a una pantalla virtual; la visualización adicional sólo cambia la presentación, no el algoritmo. Las librerías X11 utilizadas para cerrar los paneles de esa pantalla se incluyen en el instalador.

## 8. Ensayo directo del motor, sin ROS

Como diagnóstico adicional, puedes ejecutar el controlador directamente sobre Gym:

```bash
./start_ros2.sh exec python3 optimization_20261115/benchmark_fast.py \
  --name repeticion_rapida_01 --vmax 20 --lat 12.5 --brake 8.25 \
  --ros-float32 --settle-seconds 4
```

Escribe en `optimization_20261115/gym_runs/repeticion_rapida_01/`, que tampoco debe existir. `summary.json` debe indicar `completed: true`, diez vueltas, `collision: false` y `reason: completed`. Este ensayo comprueba también cuatro segundos simulados de frenado posterior. **No sustituye la validación ROS 2** del apartado anterior.

## 9. Controlador y medición

El perfil rápido selecciona rayos delanteros ±90°, infla obstáculos con un margen de 0,40 m y encuentra el intervalo libre más ancho por encima de 1,5 m. Suaviza con 31 rayos, multiplica el ángulo objetivo por 0,7 y filtra la dirección con 60 % de la orden nueva y 40 % de la anterior. La dirección se limita a ±0,4189 rad.

La orden de velocidad queda limitada por giro y distancia frontal:

```text
curvatura = abs(tan(dirección)) / 0,3302
v = min(20,
        sqrt(12,5 / max(curvatura, 0,001)),
        sqrt(2 * 8,25 * max(0, distancia_frontal - 0,50)))
```

**20 m/s es un techo de la orden, no la velocidad mantenida.** Estos coeficientes no cambian la física del simulador. Distancia frontal inferior a 0,30 m o ausencia de hueco ordenan parar. Tras la décima vuelta se sigue corrigiendo la dirección durante el frenado. Los escaneos ROS con el mismo timestamp reutilizan la orden sin avanzar otra vez el filtro.

La puerta de meta está en `(0,5; 15)` m, rumbo −π/2, semiancho 2,5 m. Cuenta cruces hacia delante después de al menos 100 m y 20 s. Saltos de posición mayores de 5 m, retrocesos de reloj o contactos invalidan la tanda. No se reinicia el vehículo entre vueltas. El paso físico es 0,01 s.

El auditor aplica otra instancia del mismo contador a los timestamps de odometría del rosbag, usando `/daniel_gap/timing_origin`. Esto es un reconteo separado, no otro reloj físico independiente. Verifica las banderas de contacto recibidas y las posiciones del centro del vehículo; esas comprobaciones tienen el alcance descrito en el JSON de auditoría.

## 10. Resultados registrados y replicación

| Ejecución nativa del autor | Vueltas | Mejor | Promedio | Total | Auditoría |
|---|---:|---:|---:|---:|---:|
| Grabación de YouTube, temporizadores normales | 10 | 50,82 s | 51,340 s | 513,40 s | 17/17 |
| Validación anterior, temporizadores normales | 10 | 50,89 s | 51,246 s | 512,46 s | 17/17 |
| Validación anterior, lockstep | 10 | 50,85 s | 51,166 s | 511,66 s | 17/17 |

Las tres tandas registraron cero contactos y parada estable posterior. Los datos de la grabación enlazada están en [su auditoría](ros2_validation/recording_20261115_wall/checks_ros.json), [CSV de vueltas](ros2_validation/recording_20261115_wall/run/laps_ros.csv) y [resumen](ros2_validation/recording_20261115_wall/run/summary_ros.json). La nueva tanda alcanzó 9,910 m/s longitudinales. En la tanda anterior de lockstep se midieron 9,943 m/s longitudinales. Es la mejor configuración ensayada; no se demuestra un mínimo global. La planificación DDS, la carga del equipo y las versiones del sistema pueden variar los tiempos. Reproducir el procedimiento y cumplir los controles es el criterio principal; no se promete obtener exactamente cada centésima publicada.

- [Informe de resultados y experimentos](RESULTADOS_MAX_VELOCIDAD.md).
- [Auditoría original con temporizadores normales](ros2_validation/fast_20261115_final_wall/checks_ros.json) y [auditoría lockstep](ros2_validation/fast_20261115_final_lockstep/checks_ros.json).

Los registros publicados son históricos y conservan las rutas y hashes del equipo donde se midieron. Los rosbags originales no forman parte del Git normal: no necesitas descargarlos para ejecutar el proyecto. Para **reauditar aquella medición exacta** sí se necesita su rosbag original; no hay todavía una Release pública con esos archivos. Las nuevas ejecuciones generan sus propios bags auditables. No reemplaces las evidencias originales con tus pruebas.

## 11. Problemas frecuentes

| Síntoma | Acción |
|---|---|
| Falta `Sakhir_map` o no coincide su hash | Importa el ZIP original del curso siguiendo el apartado 4; no cambies escala, origen ni imagen. |
| No se encuentra `.ros2_ws` o algún paquete | Ejecuta `bash installation/setup_workspace.sh` y revisa su salida de colcon. |
| Error al instalar Gym 0.19 | Usa el bootstrap, Python 3.10 y sus versiones fijadas de pip/setuptools/wheel; no actualices herramientas por tu cuenta. |
| `ModuleNotFoundError`, NumPy o Numba incompatibles | Usa `./start_ros2.sh exec ...`; evita Conda y entornos Python ajenos. Ejecuta `./start_ros2.sh check`. |
| Pantalla `:111` ocupada | Elige otra, por ejemplo `--display :112`. No borres locks de otra sesión. |
| Interferencia con otra simulación | Elige otro `--domain` libre. Para demo/status, usa el mismo `ROS_DOMAIN_ID` en ambas terminales. |
| Carpeta de salida ya existente | Cambia su nombre; el runner no borra ejecuciones anteriores. |
| Videos falla al abrir un MP4 válido | Usa el comando `ffplay` anterior. Si falla la aceleración gráfica, prueba `LIBGL_ALWAYS_SOFTWARE=1 ffplay -autoexit ruta/al/video.mp4`; verifica antes la decodificación completa con FFmpeg. |
| RViz negro o error OpenGL | Prueba `LIBGL_ALWAYS_SOFTWARE=1 ./start_ros2.sh demo`. El runner ya utiliza renderizado por software. |
| Timeout o fallos de procesos | Revisa `manifest.json` y `logs/`; cierra otras simulaciones. Puedes aumentar `--timeout-wall`, pero no considerar válida una tanda incompleta. |
| Error de APT | Detén el procedimiento y resuelve el error mostrado. Los scripts de reparación históricos no son un paso normal de instalación. |

Para cambiar código Python, el workspace usa instalación con enlaces. Si cambias `setup.py`, entrypoints, metadatos o recursos instalados, vuelve a ejecutar `bash installation/setup_workspace.sh`. Si cambias control, cronometraje, mapa o física, realiza una nueva tanda completa y su auditoría.

## Estructura y procedencia

`daniel_gap/` contiene el controlador y el contador; `config/` su perfil; `tests/` las pruebas del núcleo; `ros2_validation/` la integración, los dos modos de grabación, el observador y el auditor; `installation/` la preparación portable. La ruta principal utiliza los lanzadores desde el checkout; no se promete que una instalación aislada de `daniel_gap` mediante pip incluya todo el proyecto.

Ver [THIRD_PARTY.md](THIRD_PARTY.md) para las fuentes y atribuciones. El código propio utiliza [licencia MIT](LICENSE); se conservan los avisos del bridge externo. Los pesos de la rúbrica presentan contradicciones y la clasificación depende de otros estudiantes y del docente; los resultados no equivalen a una nota otorgada.
