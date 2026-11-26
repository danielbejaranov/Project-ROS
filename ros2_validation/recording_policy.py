"""Shared recording and telemetry rules; deliberately independent of ROS nodes."""
import math


TELEMETRY_COLUMNS = (
    'sim_time', 'x', 'y', 'yaw', 'measured_vx', 'measured_vy', 'command_speed', 'laps',
)


def positive_stride(value):
    """Accept an explicit positive integer from a constructor or argparse."""
    number = int(value)
    if isinstance(value, bool) or number < 1 or (not isinstance(value, str) and number != value):
        raise ValueError('scan_stride must be a positive integer')
    return number


def selected_scan(received_number, scan_stride=1):
    """Retain every scan by default; a stride retains the first, then every Nth."""
    return (received_number - 1) % scan_stride == 0


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def bag_timestamp_ns(topic, message, latest_clock_ns, header_topics):
    """Use sensor acquisition time, the Clock value itself, or latest received Clock."""
    if topic == '/clock':
        timestamp = stamp_ns(message.clock)
    elif topic in header_topics:
        timestamp = stamp_ns(message.header.stamp)
    else:
        timestamp = int(latest_clock_ns)
    return max(0, timestamp)


def telemetry_row(message, command_speed, laps):
    """Measured model velocities and last command remain separate columns."""
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    velocity = message.twist.twist.linear
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return [stamp_ns(message.header.stamp) / 1e9, p.x, p.y, yaw,
            float(velocity.x), float(velocity.y), float(command_speed), laps]


def console_due(sim_time, previous_time, interval=5.):
    return previous_time is None or sim_time < previous_time or sim_time - previous_time >= interval - 1e-8


def progress_line(vehicle, sim_time, laps, x, y, measured_vx, stored_scans, received_scans, collision):
    return (f't={sim_time:.2f}s {vehicle.upper()} lap={laps}/10 '
            f'x={x:.3f}m y={y:.3f}m vx={measured_vx:.3f}m/s '
            f'scans={stored_scans}/{received_scans} stored/received '
            f'collision={"YES" if collision else "NO"}')


def recording_metadata(scan_stride, scan_topics, header_topics):
    full_scan = scan_stride == 1
    return {
        'recording_profile': 'validation_full_received' if full_scan else 'validation_scan_stride',
        'scan_stride': scan_stride,
        'scan_topics': sorted(scan_topics),
        'bag_scan_sampling': ('all received LaserScan messages (stride=1)' if full_scan else
                              f'first and every {scan_stride}th received LaserScan per scan topic'),
        'bag_timestamp_policy': {
            'unit': 'ns of simulation time',
            'header_stamp_topics': sorted(header_topics),
            '/clock': 'message.clock, including the current message',
            'other_topics': 'latest received /clock; zero before the first clock',
        },
        'telemetry_schema': {
            'version': 2,
            'columns': list(TELEMETRY_COLUMNS),
            'received_odom_stride': 10,
            'units': {'sim_time': 's', 'x': 'm', 'y': 'm', 'yaw': 'rad',
                      'measured_vx': 'm/s', 'measured_vy': 'm/s', 'command_speed': 'm/s',
                      'laps': 'count'},
            'semantics': {
                'sim_time': 'Odometry.header.stamp',
                'position': 'Odometry.pose.pose.position in header.frame_id',
                'measured_vx': 'Odometry.twist.twist.linear.x; measured model velocity, not a speed command',
                'measured_vy': 'Odometry.twist.twist.linear.y; velocity in child_frame_id',
                'command_speed': 'last received AckermannDriveStamped.drive.speed; held value, not measured velocity',
                'sampling': 'CSV is a preview: first and every tenth received odometry; bag retains every received odometry',
            },
        },
        'coverage_note': 'Counters distinguish received DDS callbacks from successfully stored messages; '
                         'they do not prove that every simulator publication reached the observer.',
    }
