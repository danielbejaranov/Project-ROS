# Historial de optimización y validaciones — Daniel Bejarano

Fecha: **15/11/2026**. Pista: **Sakhir**. Validación en ROS 2 Humble instalado por APT.

La grabación de entrega es posterior a las dos tandas históricas descritas aquí: completó diez vueltas sin contactos registrados, con mejor vuelta de **50,82 s**, media de **51,340 s** y auditoría **17/17**. [Video en YouTube](https://youtu.be/oMR08ioytbE) y [resultados de esa grabación](ros2_validation/recording_20261115_wall/RESULTADOS.md).

En la validación anterior, la configuración rápida completó dos tandas separadas de diez vueltas consecutivas sin contactos registrados. Con temporizadores normales: **mejor 50,89 s**, total **08:32,46**. El modo lockstep obtuvo **50,85 s**. Ambas aprobaron **17/17 controles**, incluida la parada posterior.

## Diez vueltas de cada tanda

| Vuelta | Temporizadores normales (s) | Lockstep (s) |
|---:|---:|---:|
| 1 | 51,49 | 51,47 |
| 2 | 50,89 | 51,14 |
| 3 | 50,91 | 51,15 |
| 4 | 51,05 | 51,10 |
| 5 | 51,39 | 51,09 |
| 6 | 51,34 | 50,85 |
| 7 | 51,16 | 51,27 |
| 8 | 51,07 | 51,36 |
| 9 | 51,63 | 51,10 |
| 10 | 51,53 | 51,13 |

## Resumen y seguridad

| Indicador | Temporizadores normales | Lockstep |
|---|---:|---:|
| Total de diez vueltas | 08:32,46 | 08:31,66 |
| Mejor vuelta | 50,89 s | 50,85 s |
| Promedio | 51,246 s | 51,166 s |
| Desviación estándar poblacional | 0,252 s | 0,160 s |
| Contactos registrados | 0 | 0 |
| Mensajes de contacto auditados | 52504 | 53710 |
| Posiciones comprobadas | 131233 | 53745 |
| Centros fuera del carril | 0 | 0 |
| Máxima velocidad longitudinal medida | 9,922 m/s | 9,943 m/s |
| Máxima orden de velocidad | 10,123 m/s | 10,119 m/s |
| Velocidad final longitudinal | 4.310e-29 m/s | 1.847e-226 m/s |
| Observación después de la meta | 4,00 s simulados | 25,60 s simulados |
| Auditoría | 17/17 | 17/17 |

Respecto a los **173,37 s** de la tanda conservadora, la mejor vuelta con temporizadores normales baja **122,48 s**, una reducción del **70,65 %**. El modelo físico y el mapa se mantuvieron. Las tandas se comparan por tiempo simulado; no por duración del reproductor.

## Qué significa máxima velocidad

Se exploraron tres límites del controlador anterior y trece variantes rápidas en Gym, conservando los fallos. Se añadió una prueba de diez vueltas y cuatro segundos de parada guiada, y después las dos validaciones ROS. El techo configurado de **20 m/s (72 km/h)** coincide con el límite del modelo. Es un máximo permitido de orden, no la velocidad alcanzada en la pista.

La máxima longitudinal medida con temporizadores normales fue **9,922 m/s (35,72 km/h)**. La ley de giro y frenado reduce la velocidad según LiDAR. Esta es la mejor configuración validada de los ensayos realizados; no demuestra un óptimo global ni permite deducir el puesto frente a los compañeros.

## Configuración seleccionada

| Parámetro | Valor |
|---|---:|
| Perfil | Follow the Gap rápido, reactivo |
| Techo de velocidad | 20 m/s |
| Ganancia de dirección | 0,7 |
| Parámetro lateral de la ley de velocidad | 12,5 m/s² |
| Parámetro de frenado de la ley de velocidad | 8,25 m/s² |
| Ventana de suavizado del hueco | 31 rayos |
| Peso de dirección nueva | 0,6 |
| Inflación angular | radio 0,40 m |
| Hueco transitable | distancia >1,5 m |
| Dirección mecánica | ±0,4189 rad |
| Paso físico / semilla | 0,01 s / 12345 |

Los parámetros lateral y de frenado regulan el controlador; no alteran fricción, masa, neumáticos ni aceleración del simulador. La dirección y velocidad se deciden con LiDAR; la odometría solo mide vueltas.

## Correcciones necesarias y descartes

- Los ensayos más agresivos chocaron o se bloquearon y no se seleccionaron. Las diferencias de precisión float32 de LaserScan/Ackermann se comprobaron antes de ROS.
- La primera tanda ROS rápida llegó a diez vueltas, pero registró contacto 0,73 s después de la meta al enderezar mientras frenaba. Está descartada y se conserva en el archivo local del autor: `fast_20261115_half_lockstep`.
- Se interrumpió la primera ejecución de pared para corregir esa parada: `fast_20261115_half_wall`, conservada localmente. Sus vueltas no se suman a las finales.
- La solución mantiene el seguimiento lateral por LiDAR con orden de velocidad cero durante el frenado. Se repitieron desde cero ambas tandas finales.
- Los escaneos repetidos no avanzan dos veces el filtro; se republica la orden. El origen de cronometraje queda registrado para excluir la espera anterior al controlador.

## Evidencias y reproducibilidad

- Temporizadores normales: `ros2_validation/fast_20261115_final_wall/`.
- Lockstep: `ros2_validation/fast_20261115_final_lockstep/`.
- En Git se publican CSV, resumen, manifiesto con configuración y hashes, auditoría, métricas físicas y comprobaciones del video. El video de entrega se proporciona mediante el enlace de YouTube anterior; no se incluyen MP4 en Git. Los rosbags SQLite y logs completos se conservan localmente; no forman parte de este repositorio. Generar una tanda nueva permite obtener su propio bag para auditarla.
- El video conserva pista, vehículo, contador y cronómetro. La ejecución lockstep está acelerada; sus segundos simulados no se miden con la duración del video.
- El observador conserva mensajes recibidos; no se presume capturar cada publicación. Se contrastan flags físicos, centros dentro del carril y reconteo separado de odometría mediante el mismo algoritmo de puerta.
- El reconteo comparte reloj y algoritmo; no constituye un cronómetro físico independiente.
- Ensayos y runner portable: `optimization_20261115/`. Configuración de demo: `config/fast_controller.yaml`. Comandos completos: `README.md`.

## Criterios de la actividad

| Criterio | Estado técnico |
|---|---|
| Controlador reactivo Follow the Gap | Implementado; sin trayectoria global |
| Pista asignada Sakhir | Usada en todos los ensayos |
| Diez vueltas consecutivas sin colisiones | Tres tandas ROS auditadas, incluida la grabación de entrega |
| Menor vuelta entre esas diez | Registrada por tanda en CSV |
| Contador y cronómetro automáticos | Implementados y visibles |
| Video de evidencia | Enlace de YouTube en el README; vehículo, contador y cronómetro visibles |
| Tutorial en GitHub | Project-ROS público; instalación, ejecución y auditoría descritas en README |
| Sustentación y puesto del grupo | Pendientes de evaluación docente |

La mayor velocidad no permite garantizar una calificación: siguen pendientes la sustentación, resultados del grupo y resolución de las contradicciones de la rúbrica.

Los resúmenes de los candidatos ensayados están en `optimization_20261115/gym_results/all_results.json`. Los registros históricos conservan sus rutas originales y no son instrucciones para ejecutar en otra computadora.
