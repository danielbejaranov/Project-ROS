"""Pose-only measurements for the two-car experiment; never steering inputs."""
from math import cos, sin, hypot

class OvertakeDetector:
    """Require an observed nearby behind-to-ahead transition in the same direction.

    Heading agreement and lateral/Euclidean separation reject distant crossings
    of Sakhir's other branches. A pass must remain ahead for 0.5 simulation s.
    This is a measurement made after/control separately from the LiDAR policy.
    """
    def __init__(self):
        self.before = None
        self.pending = None
        self.events = []
        self.last_stamp = None

    def update(self, stamp, ego, opp):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        dx, dy = ego[0] - opp[0], ego[1] - opp[1]
        distance = hypot(dx, dy)
        alignment = cos(ego[2] - opp[2])
        along = dx*cos(opp[2]) + dy*sin(opp[2])
        lateral = -dx*sin(opp[2]) + dy*cos(opp[2])
        close = distance < 10.0 and alignment > .75 and abs(lateral) < 3.5
        sample = {'sim_time': stamp, 'ego': list(ego), 'opp': list(opp),
                  'centre_distance': distance, 'relative_longitudinal': along,
                  'relative_lateral': lateral, 'heading_cosine': alignment}
        if close and along < -1.0:
            self.before = sample
            self.pending = None
        elif close and along > 1.0 and self.before:
            if stamp - self.before['sim_time'] > 20.0:
                self.before = self.pending = None
                return
            if self.pending is None:
                self.pending = sample
            elif stamp - self.pending['sim_time'] >= .5:
                self.events.append({'before': self.before, 'ahead': self.pending,
                                    'confirmed': sample,
                                    'criterion': 'nearby same-direction behind<-1m to ahead>1m sustained0.5s'})
                self.before = self.pending = None
        elif self.pending and (not close or along <= 1.0):
            self.pending = None
