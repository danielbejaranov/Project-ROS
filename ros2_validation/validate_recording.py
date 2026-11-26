#!/usr/bin/env python3
"""Audit recording completeness, sensor stamps and measured telemetry offline.

This complements the physical/lap auditors. It never starts a ROS node or a
simulator and refuses a bag whose recording has not been closed. Counts refer
to received DDS callbacks, not to all publications made by the simulator.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path


VEHICLES = {
    'ego': {'scan': '/scan', 'odom': '/ego_racecar/odom', 'drive': '/drive',
            'base': 'ego_racecar/base_link', 'laser': 'ego_racecar/laser'},
    'opp': {'scan': '/opp_scan', 'odom': '/opp_racecar/odom', 'drive': '/opp_drive',
            'base': 'opp_racecar/base_link', 'laser': 'opp_racecar/laser'},
}
CSV_COLUMNS = ('sim_time', 'x', 'y', 'measured_vx', 'measured_vy', 'command_speed')


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Invalid nonnegative simulation timestamp')
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def integer_counts(value):
    return (isinstance(value, dict) and bool(value)
            and all(isinstance(topic, str) and isinstance(count, int)
                    and not isinstance(count, bool) and count >= 0
                    for topic, count in value.items()))


@dataclass
class Metric:
    count: int = 0
    invalid: int = 0
    minimum: float | None = None
    maximum: float | None = None
    first: dict | None = None
    last: dict | None = None

    def add(self, value, timestamp):
        value = float(value)
        if not math.isfinite(value):
            self.invalid += 1
            return
        sample = {'timestamp_ns': int(timestamp), 'sim_time': timestamp / 1e9,
                  'value': value}
        self.count += 1
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        if self.first is None:
            self.first = sample
        self.last = sample

    def report(self, topic, field, frame, unit='m/s'):
        return {'source_topic': topic, 'source_field': field, 'unit': unit,
                'frame': frame, 'count': self.count, 'invalid_values': self.invalid,
                'min': self.minimum, 'max': self.maximum,
                'first': self.first, 'last': self.last}


def valid_scan(message):
    """Validate sensor geometry/metadata, independently of raw range validity.

    ROS LaserScan permits invalid individual returns. Official simulator noise
    can also place a finite reading outside its declared range after clipping.
    Preserve and classify these values rather than rejecting the recording.
    """
    angles = (message.angle_min, message.angle_max, message.angle_increment)
    bounds = (message.range_min, message.range_max)
    if not finite(angles + bounds + (message.time_increment, message.scan_time)):
        return False
    span = float(message.angle_max - message.angle_min)
    increment = float(message.angle_increment)
    if not (-2 * math.pi <= message.angle_min < message.angle_max <= 2 * math.pi
            and 0 < span <= 2 * math.pi and 0 < increment <= span
            and 0 <= message.range_min < message.range_max
            and message.time_increment >= 0 and message.scan_time >= 0
            and len(message.ranges) == 1080):
        return False
    # The bridge uses FOV/N; ROS producers also commonly use FOV/(N-1).
    if abs(span - increment * (len(message.ranges) - 1)) > increment * 1.1 + 1e-6:
        return False
    return True


def range_counts():
    return dict.fromkeys(('total', 'finite_in_range', 'finite_out_of_range',
                         'finite_below_range', 'finite_above_range',
                         'finite_unclassified_invalid_bounds',
                         'positive_infinity_no_return', 'nan_invalid_return',
                         'negative_infinity_invalid_return'), 0)


def classify_ranges(message, counts):
    """Count raw sensor returns; never clip, replace or modify the message."""
    bounds_valid = finite((message.range_min, message.range_max)) and message.range_min < message.range_max
    for value in message.ranges:
        counts['total'] += 1
        if math.isnan(value):
            counts['nan_invalid_return'] += 1
        elif math.isinf(value):
            key = 'positive_infinity_no_return' if value > 0 else 'negative_infinity_invalid_return'
            counts[key] += 1
        elif not bounds_valid:
            counts['finite_unclassified_invalid_bounds'] += 1
        elif value < message.range_min:
            counts['finite_out_of_range'] += 1
            counts['finite_below_range'] += 1
        elif value > message.range_max:
            counts['finite_out_of_range'] += 1
            counts['finite_above_range'] += 1
        else:
            counts['finite_in_range'] += 1


def read_csv(path):
    """Keep only the small CSV preview, never a second copy of the full bag."""
    report = {'path': str(path), 'columns': [], 'rows': 0,
              'required_columns_present': False, 'numeric_finite': False,
              'rows_matched_odometry': 0}
    samples = {}
    if not path.is_file():
        report['error'] = 'Missing telemetry CSV'
        return report, samples
    valid = True
    with path.open(newline='') as source:
        reader = csv.DictReader(source)
        report['columns'] = reader.fieldnames or []
        report['required_columns_present'] = set(CSV_COLUMNS) <= set(report['columns'])
        for row in reader:
            report['rows'] += 1
            try:
                values = {key: float(row[key]) for key in CSV_COLUMNS}
                if not finite(values.values()) or values['sim_time'] < 0:
                    raise ValueError('Non-finite or negative simulation time')
                key = round(values['sim_time'] * 1e9)
                samples.setdefault(key, []).append({'values': values, 'matched': False})
            except (KeyError, TypeError, ValueError, OverflowError):
                valid = False
    report['numeric_finite'] = valid and report['required_columns_present'] and report['rows'] > 0
    return report, samples


def bag_snapshot(bag):
    return {str(path.relative_to(bag)): [path.stat().st_size, path.stat().st_mtime_ns]
            for path in bag.rglob('*') if path.is_file()}


def require_closed(run):
    """Summary plus finalized metadata/manifest protect against live readers."""
    summaries = [path for path in (run / 'summary_ros.json', run / 'summary_dynamic.json')
                 if path.is_file()]
    if len(summaries) != 1:
        raise ValueError('Expected exactly one finalized summary_ros.json or summary_dynamic.json')
    bag = run / 'rosbag'
    if not (bag / 'metadata.yaml').is_file() or not list(bag.glob('*.db3')):
        raise ValueError('Missing closed sqlite3 bag metadata/database; do not audit a live recording')
    if any(path.name.endswith(('-wal', '-journal')) for path in bag.rglob('*')):
        raise ValueError('SQLite write sidecar present; wait until the recorder is closed')
    manifest_path = run.parent / 'manifest.json'
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        if not manifest.get('ended_utc') or manifest.get('status') in {
                'starting', 'running', 'awaiting_preflight_review'}:
            raise ValueError('Run manifest has not been finalized; do not audit a live recording')
    return summaries[0], bag


def audit(run):
    summary_path, bag = require_closed(run)
    summary = json.loads(summary_path.read_text())
    names = ('ego', 'opp') if summary_path.name == 'summary_dynamic.json' else ('ego',)
    stride = summary.get('scan_stride')
    valid_stride = isinstance(stride, int) and not isinstance(stride, bool) and stride >= 1
    received = summary.get('received_messages')
    stored = summary.get('stored_messages')
    before = bag_snapshot(bag)
    # Lazy imports keep helpers and --help usable without starting ROS or DDS.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from rosgraph_msgs.msg import Clock

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    counts = {topic: 0 for topic in types}
    cars = {}
    for name in names:
        topics = VEHICLES[name]
        csv_path = run / ('telemetry_ros.csv' if len(names) == 1 else f'telemetry_{name}.csv')
        csv_report, samples = read_csv(csv_path)
        cars[name] = {'topics': topics, 'csv': csv_report, 'csv_samples': samples,
                      'metrics': {key: Metric() for key in ('measured_vx', 'measured_vy', 'command_speed')},
                      'position_metrics': {key: Metric() for key in ('x', 'y')},
                      'scan_range_counts': range_counts(),
                      'position_finite': True, 'velocity_finite': True,
                      'scan_valid': True, 'odom_stamps_match': True, 'scan_stamps_match': True,
                      'odom_frames': set(), 'child_frames': set(), 'scan_frames': set(),
                      'command_frames': set(), 'scan_lengths': set()}
    clock_matches = True
    timestamp_valid = True
    decoded = {topic: 0 for topic in types}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == '/clock':
            message = deserialize_message(data, Clock)
            decoded[topic] += 1
            try:
                clock_matches &= stamp_ns(message.clock) == timestamp
            except ValueError:
                timestamp_valid = False
            continue
        for car in cars.values():
            topics = car['topics']
            if topic not in (topics['scan'], topics['odom'], topics['drive']):
                continue
            if topic == topics['drive']:
                message = deserialize_message(data, AckermannDriveStamped)
                car['command_frames'].add(message.header.frame_id)
                car['metrics']['command_speed'].add(message.drive.speed, timestamp)
            else:
                cls = LaserScan if topic == topics['scan'] else Odometry
                message = deserialize_message(data, cls)
                try:
                    stamp = stamp_ns(message.header.stamp)
                except ValueError:
                    timestamp_valid = False
                    stamp = -1
                if topic == topics['scan']:
                    car['scan_stamps_match'] &= stamp == timestamp
                    car['scan_frames'].add(message.header.frame_id)
                    car['scan_lengths'].add(len(message.ranges))
                    car['scan_valid'] &= valid_scan(message)
                    classify_ranges(message, car['scan_range_counts'])
                else:
                    car['odom_stamps_match'] &= stamp == timestamp
                    car['odom_frames'].add(message.header.frame_id)
                    car['child_frames'].add(message.child_frame_id)
                    p, q = message.pose.pose.position, message.pose.pose.orientation
                    linear, angular = message.twist.twist.linear, message.twist.twist.angular
                    car['position_finite'] &= finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    car['position_metrics']['x'].add(p.x, timestamp)
                    car['position_metrics']['y'].add(p.y, timestamp)
                    car['velocity_finite'] &= finite((linear.x, linear.y, linear.z,
                                                     angular.x, angular.y, angular.z))
                    car['metrics']['measured_vx'].add(linear.x, timestamp)
                    car['metrics']['measured_vy'].add(linear.y, timestamp)
                    for candidate in car['csv_samples'].get(stamp, []):
                        values = candidate['values']
                        candidate['matched'] |= all(math.isclose(values[key], actual,
                            rel_tol=1e-8, abs_tol=1e-8) for key, actual in (
                                ('x', p.x), ('y', p.y), ('measured_vx', linear.x),
                                ('measured_vy', linear.y)))
            decoded[topic] = decoded.get(topic, 0) + 1
    del reader
    checks = {
        'bag_unchanged_during_audit': before == bag_snapshot(bag),
        'scan_stride_declared_positive_integer': valid_stride,
        'summary_received_counts_valid': integer_counts(received),
        'summary_stored_counts_valid': integer_counts(stored),
        'summary_stored_counts_match_bag': integer_counts(stored) and stored == counts,
        'clock_messages_present': counts.get('/clock', 0) > 0,
        'clock_type_correct': types.get('/clock') == 'rosgraph_msgs/msg/Clock',
        'clock_values_equal_bag_timestamps': clock_matches,
        'sensor_timestamps_valid': timestamp_valid,
    }
    counter_valid = integer_counts(received) and integer_counts(stored)
    checks['stored_never_exceeds_received'] = bool(counter_valid and set(received) == set(stored)
        and all(stored[topic] <= received[topic] for topic in stored))
    scan_topics = {VEHICLES[name]['scan'] for name in names}
    checks['declared_scan_topics_match_vehicles'] = set(summary.get('scan_topics', [])) == scan_topics
    checks['non_scan_messages_all_received_stored'] = bool(counter_valid
        and set(received) == set(stored)
        and all(stored[topic] == received[topic] for topic in stored if topic not in scan_topics))
    checks['scan_counts_match_declared_stride'] = bool(counter_valid and valid_stride
        and all(topic in stored and topic in received
                and stored[topic] == (received[topic] + stride - 1) // stride
                for topic in scan_topics))
    if stride == 1:
        checks['every_received_scan_stored'] = bool(counter_valid
            and all(topic in stored and stored[topic] == received.get(topic) for topic in scan_topics))
    results = {}
    for name, car in cars.items():
        topics = car['topics']
        for kind, typename in (('scan', 'sensor_msgs/msg/LaserScan'),
                               ('odom', 'nav_msgs/msg/Odometry'),
                               ('drive', 'ackermann_msgs/msg/AckermannDriveStamped')):
            checks[f'{name}_{kind}_messages_present'] = counts.get(topics[kind], 0) > 0
            checks[f'{name}_{kind}_type_correct'] = types.get(topics[kind]) == typename
        checks.update({
            f'{name}_odom_header_stamp_equals_bag_stamp': car['odom_stamps_match'],
            f'{name}_scan_header_stamp_equals_bag_stamp': car['scan_stamps_match'],
            f'{name}_odom_map_frame': car['odom_frames'] == {'map'},
            f'{name}_odom_vehicle_child_frame': car['child_frames'] == {topics['base']},
            f'{name}_scan_laser_frame': car['scan_frames'] == {topics['laser']},
            f'{name}_positions_and_orientation_finite': car['position_finite'],
            f'{name}_measured_velocities_finite': car['velocity_finite'],
            f'{name}_scan_geometry_and_length_valid': car['scan_valid'] and car['scan_lengths'] == {1080},
            f'{name}_command_speed_finite': car['metrics']['command_speed'].invalid == 0,
            f'{name}_csv_required_columns': car['csv']['required_columns_present'],
            f'{name}_csv_numeric_finite': car['csv']['numeric_finite'],
        })
        matched = sum(row['matched'] for rows in car['csv_samples'].values() for row in rows)
        car['csv']['rows_matched_odometry'] = matched
        expected_rows = (counts.get(topics['odom'], 0) + 9) // 10
        car['csv']['expected_rows_at_received_odom_stride_10'] = expected_rows
        checks[f'{name}_csv_rows_match_recorded_odometry'] = (car['csv']['rows'] > 0
            and matched == car['csv']['rows'])
        checks[f'{name}_csv_preview_sample_count'] = car['csv']['rows'] == expected_rows
        metrics = {}
        for key, field in (('measured_vx', 'twist.twist.linear.x'),
                           ('measured_vy', 'twist.twist.linear.y')):
            metrics[key] = car['metrics'][key].report(topics['odom'], field, topics['base'])
        metrics['command_speed'] = car['metrics']['command_speed'].report(
            topics['drive'], 'drive.speed', 'vehicle longitudinal speed command; header frames recorded below')
        results[name] = {'topics': {key: topics[key] for key in ('scan', 'odom', 'drive')},
                         'frames_recorded': {key: sorted(car[key]) for key in (
                             'odom_frames', 'child_frames', 'scan_frames', 'command_frames')},
                         'position_frame': 'map', 'velocity_frame': topics['base'],
                         'position': {axis: metric.report(topics['odom'], f'pose.pose.position.{axis}', 'map', unit='m')
                                      for axis, metric in car['position_metrics'].items()},
                         'variables': metrics, 'telemetry_csv': car['csv'],
                         'scan_lengths': sorted(car['scan_lengths']),
                         'scan_range_samples': car['scan_range_counts'],
                         'scan_range_note': 'Raw returns are classified without clipping or replacement. Out-of-range finite values, NaN and negative infinity are sensor-invalid returns; positive infinity means no return. They do not by themselves invalidate recording integrity.'}
    return {'passed': all(checks.values()), 'checks': checks, 'run': str(run),
            'summary': str(summary_path), 'storage_id': 'sqlite3', 'scan_stride': stride,
            'recording_profile': summary.get('recording_profile'),
            'counts': {'bag': counts, 'summary_stored': stored, 'summary_received': received},
            'vehicles': results, 'audited_utc': datetime.now(timezone.utc).isoformat(),
            'limitations': [
                'This checks messages received/stored by the observer; it cannot recover or count simulator publications lost before DDS callbacks.',
                'Measured vx/vy are read from Odometry.twist.twist.linear.x/y, never inferred from Ackermann commands or differentiated positions.',
                'Frames are checked as published by the bridge; this audit does not rotate velocities into another frame.',
                'LaserScan range validity is reported separately from recording integrity. The official sensor noise can produce returns beyond range_max; raw values remain unchanged.',
                'CSV is a first/every-tenth received odometry preview; command_speed is a held last-received command, not measured velocity.',
                'Recording integrity does not replace the separate lap, collision, map and overtaking auditors.',
            ]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path, help='Closed run/ directory with summary and rosbag/')
    parser.add_argument('--output', required=True, type=Path, help='New JSON; existing files are never replaced')
    args = parser.parse_args()
    run = args.run.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error('--output already exists; choose a new report path')
    if output.is_relative_to(run / 'rosbag'):
        parser.error('--output must be outside the evidence bag directory')
    try:
        result = audit(run)
    except Exception as error:
        result = {'passed': False, 'checks': {'audit_completed': False}, 'run': str(run),
                  'error': f'{type(error).__name__}: {error}',
                  'limitations': ['No running ROS node or simulator was started. Use only a finalized recording.']}
    result['auditor_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as target:
        json.dump(result, target, indent=2, ensure_ascii=False, allow_nan=False)
        target.write('\n')
    approved = sum(result['checks'].values())
    print(f"recording passed={result['passed']} checks={approved}/{len(result['checks'])} output={output}")
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
