#!/usr/bin/env python3
"""Audita diez vueltas y el paso ordenado por O1–O5 en cada vuelta del bag."""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('native_audit', ROOT/'ros2_validation/validate_run.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def coverage(positions, origins, laps, obstacles, distance=3.):
    """Odometry is evidence only; this function never commands a vehicle."""
    if not origins:
        return []
    start = origins[0]
    boundaries = start + np.r_[0., np.cumsum(laps)]
    points = np.asarray(positions, dtype=float).reshape((-1, 3))
    result = []
    for i, (begin, end) in enumerate(zip(boundaries, boundaries[1:]), 1):
        segment = points[(points[:, 0] >= begin-.001) & (points[:, 0] <= end+.001)]
        next_index = 0
        events = []
        for stamp, x, y in segment:
            if next_index == len(obstacles):
                break
            obstacle = obstacles[next_index]
            if np.hypot(x-obstacle['x'], y-obstacle['y']) <= distance:
                events.append({'obstacle': obstacle['id'], 'stamp': float(stamp)})
                next_index += 1
        result.append({'lap': i, 'passed_all_five_in_order': next_index == 5, 'events': events})
    return result


def main():
    args = base.parse_args()
    if not any(value == '--map-yaml' or value.startswith('--map-yaml=') for value in sys.argv[1:]):
        args.map_yaml = ROOT/'results/Sakhir_obstacles_5.yaml'
    if args.output.exists():
        raise FileExistsError('No se sobrescribe una auditoría previa.')
    report = base.audit(args)
    manifest = json.loads((args.run.resolve().parent/'manifest.json').read_text())
    image_hash = hashlib.sha256(Path(report['map']['image']).read_bytes()).hexdigest()
    report['checks']['audit_map_matches_recorded_map'] = image_hash == manifest.get('map_image_sha256')
    report['audited_map_sha256'] = image_hash
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run.resolve()/'rosbag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    positions, origins = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/daniel_gap/timing_origin':
            origins.append(float(deserialize_message(data, Float64).data))
        elif topic == '/ego_racecar/odom':
            msg = deserialize_message(data, Odometry)
            positions.append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                              msg.pose.pose.position.x, msg.pose.pose.position.y])
    obstacles = yaml.safe_load((HERE/'static_obstacles.yaml').read_text())['obstacles']
    visits = coverage(positions, origins, report['csv_lap_times'], obstacles)
    report['checks']['each_lap_passes_all_five_obstacles_in_order'] = (
        len(visits) == args.target_laps and all(v['passed_all_five_in_order'] for v in visits))
    report['obstacle_coverage'] = visits
    report['coverage_radius_m'] = 3.
    report['passed'] = all(report['checks'].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'passed': report['passed'], 'checks': report['checks'],
                      'best_lap': report['best_lap'], 'coverage_laps': len(visits)}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
