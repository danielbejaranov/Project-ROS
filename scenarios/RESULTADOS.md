# Resultados de la segunda parte — 22 de noviembre de 2026

Ambos escenarios se ejecutaron en ROS 2 Humble nativo, con F1TENTH Gym, DDS, RViz y rosbag. La [guía de ejecución](README.md) permite generar los mapas y repetir las tandas. Se utilizó el motor fijado en `installation/simulator-source.json`, con paso físico de 0,01 s y sin modificar parámetros físicos.

| Prueba / vehículo | Vueltas reales | Contactos registrados | Mejor vuelta | Media primeras 10 | Total primeras 10 |
|---|---:|---:|---:|---:|---:|
| Cinco obstáculos | 10 | 0 | 84.27 s | 84.449 s | 844.49 s |
| Dinámica / ego | 13 | 0 | 93.01 s | 94.407 s | 944.07 s |
| Dinámica / segundo robot | 10 | 0 | 124.87 s | 125.603 s | 1256.03 s |

El segundo vehículo comenzó exactamente 5 m delante. El auditor reconstruyó **cuatro adelantamientos**, el primero confirmado a los **18,66 s** simulados. El ego completó tres vueltas adicionales mientras el segundo alcanzaba diez; ambos mantuvieron su controlador hasta el frenado conjunto. Las trece vueltas del ego figuran en el contador, CSV y auditoría.

## Tiempos por vuelta

Todos los tiempos están en segundos de `/clock`.

| Vuelta | Cinco obstáculos | Dinámica / ego | Dinámica / segundo |
|---|---:|---:|---:|
| 1 | 84.76 | 96.88 | 124.87 |
| 2 | 84.37 | 93.17 | 125.51 |
| 3 | 84.43 | 93.16 | 125.52 |
| 4 | 84.27 | 93.72 | 126.03 |
| 5 | 84.56 | 96.76 | 125.51 |
| 6 | 84.35 | 93.24 | 125.54 |
| 7 | 84.52 | 93.01 | 126.03 |
| 8 | 84.47 | 93.88 | 125.51 |
| 9 | 84.45 | 97.14 | 125.52 |
| 10 | 84.31 | 93.11 | 125.99 |
| 11 | — | 93.31 | — |
| 12 | — | 93.60 | — |
| 13 | — | 97.31 | — |

## Evidencia verificable

- [Auditoría estática: 19/19 controles](evidence/static_20261122/checks.json), [CSV](evidence/static_20261122/laps_ros.csv), [resumen ROS](evidence/static_20261122/summary.json) y [procedencia de fuentes](evidence/static_20261122/source_provenance.json).
- [Auditoría dinámica final: 49/49 controles](evidence/dynamic_20261122/checks_final.json), [auditoría inicial](evidence/dynamic_20261122/checks.json), CSV de [ego](evidence/dynamic_20261122/laps_ego.csv) y [segundo](evidence/dynamic_20261122/laps_opp.csv), y [resumen ROS](evidence/dynamic_20261122/summary.json).
- Pruebas unitarias y regresiones: **13 aprobadas**. Los comandos y los valores predeterminados de los escenarios también se revisaron sin iniciar simulaciones.

La estática exige paso ordenado junto a los cinco obstáculos en cada vuelta. En la dinámica se aplican los mismos cinco sectores como referencias de medición sobre el mapa original, sin añadir obstáculos estáticos: las **23 vueltas completas** pasaron la comprobación de recorrido. Se verificaron ambos flags del motor, centros de odometría dentro del carril, velocidades y dirección limitadas, salida a cinco metros y parada final. El monitor dinámico observó 4,00 s simulados detenido; el reconteo con pares de odometría registró 3,98 s dentro de la tolerancia declarada de 0,05 s.

Estas carpetas contienen copias compactas de los informes originales. Las rutas absolutas de sus manifests describen la máquina de medición; no son rutas necesarias para ejecutar el proyecto en otro equipo. Los bags originales se conservan localmente y no están incluidos en Git. El reconteo comparte reloj y algoritmo con el controlador; los controles de colisión se aplican a los mensajes recibidos, y la comprobación de carril evalúa centros de odometría.

Durante la tanda dinámica el proceso supervisor de la herramienta terminó; los procesos separados del simulador, controladores, monitor y captura siguieron activos. Se volvió a adjuntar la supervisión a esos mismos procesos para cerrarlos al terminar. El [manifest](evidence/dynamic_20261122/manifest.json) registra el incidente. El reloj fue monótono, las vueltas no se reiniciaron y la grabación fue continua. Las tentativas anteriores que no completaron las condiciones se conservaron localmente como diagnósticos y no se presentan como tandas válidas.

## Videos para entregar

Se grabaron directamente las ventanas reales de RViz, con coche, contador y cronómetro visibles. Los archivos finales son `01_Sakhir_5_obstaculos_ROS2.mp4` y `02_Sakhir_adelantamiento_ROS2.mp4`. Las copias compatibles son H.264 Baseline, 1920×1080, 15 fps, con una pista AAC silenciosa. Conservan todos los fotogramas y la duración original de la captura. Consulta las verificaciones de [video estático](evidence/static_20261122/video_qa.json) y [video dinámico](evidence/dynamic_20261122/video_qa.json).

Estas tandas usan `lockstep`: el reloj del simulador puede avanzar más rápido que el tiempo de pared. El MP4 conserva el tiempo de pared capturado y sus tiempos de vuelta son simulados. No se aplicó aceleración de edición, cortes ni reconstrucción visual desde CSV.

Grabaciones actualizadas del **26 de noviembre de 2026**, con consola visible y una vista de seguimiento más amplia: [cinco obstáculos y diez vueltas](https://youtu.be/4yqzK4zvlZ8) y [dos robots y adelantamiento](https://youtu.be/xx3N9OuGNBI). Los rosbags y las auditorías de estas nuevas ejecuciones están en la [Release del 26 de noviembre](https://github.com/danielbejaranov/Project-ROS/releases/tag/evidencia-20261126). Las tablas y las evidencias anteriores documentan las tandas del **22 de noviembre**.

## Relación con la rúbrica

| Criterio del adjunto | Evidencia preparada |
|---|---|
| Cinco obstáculos y diez vueltas sin colisión | Mapa reproducible, diez vueltas completas, auditoría estática y video real |
| Otro robot 5 m delante, adelantamiento y diez vueltas por ambos | Dos instancias independientes del FTG de la primera parte, separación inicial medida, cuatro pases y 13/10 vueltas |
| Tutorial GitHub — 3 puntos | Enfoque, estructura, instalación, parámetros, ejecución y auditorías documentados |
| Contador — 2 puntos | Contadores visibles, CSV y reconteo de cada vuelta |
| Cronómetro — 3 puntos | Tiempo actual y mejor vuelta en RViz; tiempos individuales en CSV |
| Video obligatorio — 2 puntos | Enlaces a las dos grabaciones reales, con RViz, contador, cronómetro y consola visibles |

Los 30 puntos de desempeño dependen del ranking del grupo y la nota final también depende de la sustentación oral. Estos resultados permiten demostrar el funcionamiento técnico; la asignación de puntos corresponde al docente.
