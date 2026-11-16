# Nueva captura real de F1TENTH Gym y RViz

Se completó una nueva ejecución del simulador nativo F1TENTH Gym con ROS 2 Humble, capturada directamente desde X11 mientras dos procesos reales de RViz mostraban el circuito y el vehículo. No se creó una animación desde un CSV.

| Medida | Resultado |
|---|---:|
| Vueltas consecutivas | 10 |
| Mejor vuelta | 50.82 s |
| Promedio | 51.340 s |
| Total de vueltas | 513.40 s |
| Auditoría | 17/17 |
| Flags de colisión positivos / recibidos | 0 / 58,252 |
| Centros de odometría fuera de pista / comprobados | 0 / 144,641 |
| Observación posterior a la meta | 4.00 s simulados |
| Velocidad longitudinal máxima medida | 9.909877 m/s |

Los tiempos proceden del reloj de simulación ROS; la duración del video corresponde a tiempo de pared. El reconteo aplica el mismo LapCounter a la odometría grabada desde el origen de cronometraje explícito. La revisión geométrica comprueba centros, no toda la carrocería.

| Vuelta | Segundos |
|---|---:|
| 1 | 51.97 |
| 2 | 51.09 |
| 3 | 51.83 |
| 4 | 51.00 |
| 5 | 51.35 |
| 6 | 50.82 |
| 7 | 51.72 |
| 8 | 51.43 |
| 9 | 50.90 |
| 10 | 51.29 |

La física, el controlador, el contador y el monitor original permanecieron intactos: sus hashes coinciden antes y después de la ejecución. Los cambios locales solo afectan la presentación en RViz y están documentados en `capture_provenance.json`.

[Video de la ejecución real en YouTube](https://youtu.be/oMR08ioytbE).

En esta carpeta se publican el manifiesto y los hashes originales, la auditoría, los CSV, el resumen y `video_check.json`. Sus rutas absolutas describen el equipo de medición; no son rutas que deba crear otra persona. El MP4 y el rosbag originales se conservan localmente y no forman parte de Git.

Para repetir una ejecución con esta presentación, sigue el apartado “Grabar con las dos vistas del video de evidencia” del [README](../../README.md). Cada persona genera y audita su propio rosbag. Reauditar exactamente la tanda histórica requiere su bag original.
