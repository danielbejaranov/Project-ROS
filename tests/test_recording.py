"""Recording regressions without DDS, node startup or simulator execution."""
import math
from types import SimpleNamespace as NS
import unittest

from ros2_validation.recording_policy import (
    TELEMETRY_COLUMNS, bag_timestamp_ns, positive_stride,
    recording_metadata, selected_scan, telemetry_row, console_due,
)


class RecordingTests(unittest.TestCase):
    def test_default_preserves_every_received_scan(self):
        self.assertTrue(all(selected_scan(number) for number in range(1, 206)))

    def test_explicit_stride_is_first_then_every_nth(self):
        selected = [n for n in range(1, 206) if selected_scan(n, 100)]
        self.assertEqual(selected, [1, 101, 201])
        for invalid in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                positive_stride(invalid)

    def test_sensor_and_clock_timestamps_do_not_use_a_stale_clock(self):
        sensor = NS(header=NS(stamp=NS(sec=12, nanosec=345)))
        clock = NS(clock=NS(sec=13, nanosec=678))
        latest = 10_000_000_000
        headers = {'/scan', '/ego_racecar/odom'}
        for topic in headers:
            self.assertEqual(bag_timestamp_ns(topic, sensor, latest, headers), 12_000_000_345)
        self.assertEqual(bag_timestamp_ns('/clock', clock, latest, headers), 13_000_000_678)
        self.assertEqual(bag_timestamp_ns('/drive', object(), latest, headers), latest)

    def test_csv_distinguishes_measured_velocity_from_a_different_command(self):
        message = NS(
            header=NS(stamp=NS(sec=8, nanosec=500_000_000)),
            pose=NS(pose=NS(position=NS(x=7., y=-4.2),
                orientation=NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5)))),
            twist=NS(twist=NS(linear=NS(x=-1.25, y=.3))),
        )
        row = dict(zip(TELEMETRY_COLUMNS, telemetry_row(message, command_speed=9., laps=2)))
        self.assertEqual(row['sim_time'], 8.5)
        self.assertEqual((row['x'], row['y']), (7., -4.2))
        self.assertAlmostEqual(row['yaw'], math.pi / 2)
        self.assertEqual((row['measured_vx'], row['measured_vy'], row['command_speed']), (-1.25, .3, 9.))

    def test_summary_discloses_decimation_and_csv_preview_units(self):
        metadata = recording_metadata(4, {'/scan', '/opp_scan'}, {'/scan', '/opp_scan', '/ego_racecar/odom'})
        self.assertEqual(metadata['scan_stride'], 4)
        self.assertEqual(metadata['recording_profile'], 'validation_scan_stride')
        self.assertEqual(metadata['telemetry_schema']['units']['measured_vx'], 'm/s')
        self.assertEqual(metadata['telemetry_schema']['received_odom_stride'], 10)

    def test_console_does_not_repeat_for_duplicate_simulation_time(self):
        self.assertTrue(console_due(0., None))
        self.assertFalse(console_due(0., 0.))
        self.assertFalse(console_due(4.99, 0.))
        self.assertTrue(console_due(5., 0.))


if __name__ == '__main__':
    unittest.main()
