"""Regresiones de seguridad y de la evidencia de adelantamiento de la parte 2."""
import math
import unittest

from daniel_gap.obstacle_core import MidpointGapController
from scenarios.dynamic_metrics import OvertakeDetector


class ObstaclePolicyTests(unittest.TestCase):
    def test_invalid_scans_stop(self):
        for value in (float('nan'), float('-inf'), 0.):
            self.assertEqual(MidpointGapController().command([value]*1080, -2.35, 4.7/1079), (0., 0.))

    def test_open_corridor_stays_forward(self):
        steer, speed = MidpointGapController(max_speed=6).command([10.]*1080, -2.35, 4.7/1079)
        self.assertLess(abs(steer), .01)
        self.assertGreater(speed, 0.)
        self.assertLessEqual(speed, 6.)

    def test_immediate_frontal_obstacle_stops(self):
        scan = [10.]*1080
        scan[530:550] = [.2]*20
        self.assertEqual(MidpointGapController().command(scan, -2.35, 4.7/1079), (0., 0.))


class OvertakeEvidenceTests(unittest.TestCase):
    @staticmethod
    def feed(detector, samples, opp=(0., 0., 0.)):
        for stamp, pose in samples:
            detector.update(stamp, pose, opp)

    def test_nearby_sustained_pass_is_recorded(self):
        detector = OvertakeDetector()
        self.feed(detector, [(0., (-5., 0., 0.)), (1., (1.5, .7, 0.)), (1.6, (2.5, .7, 0.))])
        self.assertEqual(len(detector.events), 1)

    def test_opposite_direction_is_not_an_overtake(self):
        detector = OvertakeDetector()
        self.feed(detector, [(0., (-5., 0., math.pi)), (1., (1.5, .7, math.pi)), (2., (2.5, .7, math.pi))])
        self.assertFalse(detector.events)

    def test_distant_crossing_is_rejected(self):
        detector = OvertakeDetector()
        self.feed(detector, [(0., (-5., 30., 0.)), (1., (1.5, 30., 0.)), (2., (2.5, 30., 0.))])
        self.assertFalse(detector.events)

    def test_brief_ahead_then_behind_is_rejected(self):
        detector = OvertakeDetector()
        self.feed(detector, [(0., (-5., 0., 0.)), (1., (1.5, .7, 0.)), (1.1, (-2., .7, 0.))])
        self.assertFalse(detector.events)

    def test_duplicate_timestamps_cannot_confirm_a_pass(self):
        detector = OvertakeDetector()
        self.feed(detector, [(0., (-5., 0., 0.)), (1., (1.5, .7, 0.)), (1., (2.5, .7, 0.))])
        self.assertFalse(detector.events)
