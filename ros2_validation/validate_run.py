#!/usr/bin/env python3
"""Audita una corrida ROS grabada sin iniciar DDS, nodos ni simuladores.

El reconteo usa otra instancia del mismo LapCounter y los timestamps de
odometría del bag. No representa un segundo reloj físico independiente.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import label
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from daniel_gap.core import LapCounter


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True,
                        help='Carpeta con laps_ros.csv, summary_ros.json y rosbag/.')
    parser.add_argument('--max-speed', type=float, required=True,
                        help='Límite de orden del candidato, en m/s; nunca se acepta más de 20.')
    parser.add_argument('--output', type=Path, required=True,
                        help='JSON nuevo de auditoría; no modifica el bag ni el CSV.')
    parser.add_argument('--map-yaml', type=Path,
                        default=PROJECT / 'results/Sakhir_map.yaml')
    parser.add_argument('--target-laps', type=int, default=10)
    parser.add_argument('--start-x', type=float, default=0.5)
    parser.add_argument('--start-y', type=float, default=15.0)
    parser.add_argument('--start-yaw', type=float, default=-math.pi / 2)
    parser.add_argument('--lap-tolerance', type=float, default=0.1,
                        help='Diferencia máxima entre tiempos CSV y reconteo odom, en segundos.')
    parser.add_argument('--min-collision-messages', type=int, default=1)
    args = parser.parse_args()
    if not math.isfinite(args.max_speed) or args.max_speed <= 0:
        parser.error('--max-speed debe ser finito y positivo.')
    if args.target_laps <= 0 or args.min_collision_messages <= 0:
        parser.error('--target-laps y --min-collision-messages deben ser positivos.')
    if not math.isfinite(args.lap_tolerance) or args.lap_tolerance < 0:
        parser.error('--lap-tolerance debe ser finita y no negativa.')
    if not all(math.isfinite(v) for v in (args.start_x, args.start_y, args.start_yaw)):
        parser.error('La puerta de meta debe tener coordenadas finitas.')
    return args


def map_regions(path):
    """Lee geometría y umbral de espacio libre del YAML de mapa ROS."""
    path = path.resolve()
    config = yaml.safe_load(path.read_text())
    image_path = Path(config['image'])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert('L'), dtype=float)
    resolution = float(config['resolution'])
    origin = [float(value) for value in config['origin']]
    free_threshold = float(config['free_thresh'])
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3:
        raise ValueError('Resolución u origen de mapa inválidos.')
    if not all(math.isfinite(value) for value in origin):
        raise ValueError('Origen del mapa no finito.')
    if not 0 <= free_threshold <= 1:
        raise ValueError('free_thresh debe estar entre 0 y 1.')
    occupancy = pixels / 255 if bool(config.get('negate', 0)) else 1 - pixels / 255
    regions, _ = label(occupancy < free_threshold)
    info = {'yaml': str(path), 'image': str(image_path.resolve()),
            'resolution': resolution, 'origin': origin,
            'free_threshold': free_threshold, 'shape': list(pixels.shape)}
    return regions, info


def grid_coordinates(x, y, info):
    ox, oy, yaw = info['origin']
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = np.asarray(x) - ox, np.asarray(y) - oy
    columns = np.floor((cosine * dx + sine * dy) / info['resolution'])
    rows = info['shape'][0] - 1 - np.floor((-sine * dx + cosine * dy) / info['resolution'])
    return rows, columns


def audit(args):
    run = args.run.resolve()
    summary = json.loads((run / 'summary_ros.json').read_text())
    with (run / 'laps_ros.csv').open(newline='') as source:
        csv_rows = list(csv.DictReader(source))
    csv_times = [float(row['seconds']) for row in csv_rows]
    summary_times = [float(value) for value in summary.get('laps', [])]
    csv_indices = [int(row['lap']) for row in csv_rows]
    counter = LapCounter(args.start_x, args.start_y, args.start_yaw)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'rosbag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    counts = {}
    positive_collision_flags = 0
    clock_previous = None
    clock_backwards = 0
    max_speed = 0.0
    max_steer = 0.0
    max_measured_vx = None
    max_abs_measured_vx = 0.0
    final_vx = None
    last_odometry = None
    last_speed = None
    recorded_numbers_finite = True
    positions = []
    odom_samples = []
    timing_origins = []
    first_odom_stamp = last_odom_stamp = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == '/daniel_gap/collision':
            if deserialize_message(data, Bool).data:
                positive_collision_flags += 1
        elif topic == '/daniel_gap/timing_origin':
            timing_origins.append(float(deserialize_message(data, Float64).data))
        elif topic == '/clock':
            message = deserialize_message(data, Clock)
            stamp = message.clock.sec * 10**9 + message.clock.nanosec
            if clock_previous is not None and stamp < clock_previous:
                clock_backwards += 1
            clock_previous = stamp
        elif topic == '/drive':
            message = deserialize_message(data, AckermannDriveStamped)
            last_speed = float(message.drive.speed)
            steer = float(message.drive.steering_angle)
            if math.isfinite(last_speed) and math.isfinite(steer):
                max_speed = max(max_speed, abs(last_speed))
                max_steer = max(max_steer, abs(steer))
            else:
                recorded_numbers_finite = False
        elif topic == '/ego_racecar/odom':
            message = deserialize_message(data, Odometry)
            p = message.pose.pose.position
            velocity = message.twist.twist.linear
            final_vx = float(velocity.x)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if first_odom_stamp is None:
                first_odom_stamp = stamp
            last_odom_stamp = stamp
            positions.append((p.x, p.y))
            odom_samples.append((stamp, p.x, p.y))
            last_odometry = {'simulation_stamp': stamp, 'x': float(p.x), 'y': float(p.y),
                             'linear_x': final_vx, 'linear_y': float(velocity.y)}
            if math.isfinite(final_vx):
                max_measured_vx = final_vx if max_measured_vx is None else max(max_measured_vx, final_vx)
                max_abs_measured_vx = max(max_abs_measured_vx, abs(final_vx))
            else:
                recorded_numbers_finite = False
            if not (math.isfinite(p.x) and math.isfinite(p.y)):
                recorded_numbers_finite = False

    # La infraestructura puede publicar odometría antes de iniciar el controlador.
    # El origen explícito permite reconstruir únicamente su intervalo de medida.
    timing_origin = None
    timing_origin_source = 'all_recorded_odometry'
    summary_origin = summary.get('timing_origin')
    if timing_origins:
        if not all(math.isfinite(value) for value in timing_origins):
            raise ValueError('El tópico timing_origin contiene valores no finitos.')
        timing_origin = timing_origins[0]
        timing_origin_source = 'rosbag:/daniel_gap/timing_origin'
        if any(abs(value - timing_origin) > 1e-8 for value in timing_origins):
            raise ValueError('El bag contiene distintos orígenes de cronometraje.')
        if summary_origin is not None and abs(float(summary_origin) - timing_origin) > 1e-8:
            raise ValueError('El origen de summary y el del bag no coinciden.')
    elif summary_origin is not None:
        timing_origin = float(summary_origin)
        timing_origin_source = 'summary:timing_origin'
    if timing_origin is not None and not math.isfinite(timing_origin):
        raise ValueError('El origen de cronometraje no es finito.')

    # Las colisiones se evalúan en el bag entero, incluso antes del origen.
    counter.invalid = positive_collision_flags > 0
    first_recount_stamp = None
    recount_messages = 0
    for stamp, x, y in odom_samples:
        if timing_origin is not None and stamp < timing_origin - 1e-8:
            continue
        if first_recount_stamp is None:
            first_recount_stamp = stamp
        recount_messages += 1
        if math.isfinite(x) and math.isfinite(y):
            counter.update(x, y, stamp)
        else:
            counter.invalid = True

    regions, map_info = map_regions(args.map_yaml)
    start_row, start_col = grid_coordinates(args.start_x, args.start_y, map_info)
    height, width = regions.shape
    if not (0 <= start_row < height and 0 <= start_col < width):
        raise ValueError('La puerta inicial queda fuera del mapa.')
    track_region = regions[int(start_row), int(start_col)]
    if track_region == 0:
        raise ValueError('La puerta inicial no está en una región libre del mapa.')
    points = np.asarray(positions, dtype=float).reshape((-1, 2))
    on_track = np.zeros(len(points), dtype=bool)
    if len(points):
        rows, columns = grid_coordinates(points[:, 0], points[:, 1], map_info)
        inside = (np.isfinite(rows) & np.isfinite(columns) &
                  (rows >= 0) & (columns >= 0) & (rows < height) & (columns < width))
        on_track[inside] = regions[rows[inside].astype(int), columns[inside].astype(int)] == track_region

    def times_agree(left, right, tolerance):
        return (len(left) == len(right) == args.target_laps and
                all(math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tolerance
                    for a, b in zip(left, right)))

    collision_messages = counts.get('/daniel_gap/collision', 0)
    enough_collision_messages = collision_messages >= args.min_collision_messages
    speed_limit = min(args.max_speed, 20.0)
    encoded_speed_limit = float(np.float32(speed_limit))
    encoded_steer_limit = float(np.float32(0.4189))
    float_tolerance = 1e-6
    finish_origin = timing_origin if timing_origin is not None else first_recount_stamp
    post_finish_seconds = None
    if (finish_origin is not None and last_odom_stamp is not None and
            len(csv_times) == args.target_laps and
            all(math.isfinite(value) and value > 0 for value in csv_times)):
        post_finish_seconds = last_odom_stamp - (finish_origin + sum(csv_times))
    checks = {
        # Los siete controles de la auditoría original conservan sus nombres.
        'ten_laps_csv_and_observer_agree': times_agree(csv_times, summary_times, 1e-6),
        'no_collision_in_entire_recorded_flag_stream': enough_collision_messages and positive_collision_flags == 0,
        'clock_monotonic': counts.get('/clock', 0) > 0 and clock_backwards == 0,
        'all_odom_centres_on_track': bool(len(points) and on_track.all()),
        'command_speed_within_limit': counts.get('/drive', 0) > 0 and max_speed <= encoded_speed_limit + float_tolerance,
        'command_steer_within_limit': counts.get('/drive', 0) > 0 and max_steer <= encoded_steer_limit + float_tolerance,
        'last_speed_command_zero': last_speed == 0.0,
        'collision_flag_messages_present': enough_collision_messages,
        'summary_completed': summary.get('completed') is True,
        'summary_reports_no_collision': summary.get('collision') is False,
        'positive_csv_lap_times': bool(csv_times) and all(math.isfinite(value) and value > 0 for value in csv_times),
        'csv_lap_indices_sequential': csv_indices == list(range(1, args.target_laps + 1)),
        'recorded_numbers_finite': recorded_numbers_finite,
        'odom_recount_not_invalid': not counter.invalid,
        'odom_recount_matches_csv': times_agree(csv_times, counter.times, args.lap_tolerance),
        'final_vehicle_stopped': final_vx is not None and math.isfinite(final_vx) and abs(final_vx) < 0.05,
        'post_finish_observed_at_least_one_sim_second': post_finish_seconds is not None and post_finish_seconds >= 1.0 - 1e-8,
    }
    deltas = ([b - a for a, b in zip(csv_times, counter.times)]
              if len(csv_times) == len(counter.times) else None)
    return {
        'passed': all(checks.values()), 'checks': checks, 'run': str(run),
        'target_laps': args.target_laps, 'bag_messages': counts,
        'positive_collision_flags': positive_collision_flags,
        'minimum_collision_messages': args.min_collision_messages,
        'odom_positions_checked': len(points), 'outside_track': int((~on_track).sum()),
        'max_commanded_speed': max_speed, 'candidate_max_speed': args.max_speed,
        'accepted_speed_limit': speed_limit, 'accepted_speed_limit_float32': encoded_speed_limit,
        'max_abs_steering': max_steer, 'accepted_steer_limit_float32': encoded_steer_limit,
        'float_tolerance': float_tolerance, 'last_commanded_speed': last_speed,
        'max_measured_longitudinal_velocity': max_measured_vx,
        'max_abs_measured_longitudinal_velocity': max_abs_measured_vx,
        'final_vx': final_vx, 'last_odometry': last_odometry,
        'post_finish_sim_seconds': post_finish_seconds,
        'speed_note': 'Las velocidades medidas proceden de Odometry.twist.twist.linear.x; '
                      'las órdenes Ackermann se registran por separado. '
                      'La condición de parada exige abs(vx) < 0.05 m/s.',
        'best_lap': min(csv_times) if csv_times else None,
        'mean_lap': sum(csv_times) / len(csv_times) if csv_times else None,
        'total_lap_time': sum(csv_times), 'csv_lap_times': csv_times,
        'summary_lap_times': summary_times, 'odom_recount_lap_times': counter.times,
        'odom_recount_deltas_seconds': deltas, 'lap_tolerance_seconds': args.lap_tolerance,
        'first_odom_timestamp': first_odom_stamp, 'last_odom_timestamp': last_odom_stamp,
        'timing_origin': timing_origin, 'timing_origin_source': timing_origin_source,
        'timing_origin_topic_messages': len(timing_origins),
        'first_recount_timestamp': first_recount_stamp,
        'recount_odom_messages': recount_messages,
        'gate': {'x': args.start_x, 'y': args.start_y, 'yaw': args.start_yaw,
                 'half_width': counter.width, 'minimum_distance': counter.min_distance,
                 'minimum_time': counter.min_time},
        'recount_note': 'Reaplica el mismo LapCounter a los timestamps de odometría del bag, '
                        'desde timing_origin si está disponible; '
                        'es un reconteo separado, no un segundo reloj físico independiente. '
                        'El resumen del observador original obtiene los tiempos del estado del controlador.',
        'coverage_note': 'La ausencia de colisiones se comprueba en los flags recibidos y guardados; '
                         'no se supone que el bag contenga cada publicación original. '
                         'La comprobación geométrica evalúa centros de odometría, no toda la carrocería.',
        'core_sha256_for_recount': hashlib.sha256((PROJECT / 'daniel_gap/core.py').read_bytes()).hexdigest(),
        'map': map_info,
    }


def main():
    args = parse_args()
    try:
        result = audit(args)
        exit_code = 0 if result['passed'] else 1
    except Exception as error:
        result = {'passed': False, 'run': str(args.run.resolve()),
                  'error': f'{type(error).__name__}: {error}'}
        exit_code = 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
