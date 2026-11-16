"""Control reactivo independiente de ROS. Unidades: m, s, rad."""
import numpy as np
try:
    from numba import njit
except ImportError:
    def njit(**kwargs):
        return lambda fn: fn

@njit(cache=True)
def inflate_obstacles(r, increment):
    """Misma erosión angular, compilada para sostener la frecuencia ROS."""
    safe = r.copy()
    for i in range(len(r)):
        half = int(np.arctan2(.40, max(r[i], .01)) / increment)
        lo, hi = max(0, i-half), min(len(r), i+half+1)
        for j in range(lo, hi):
            if r[i] < safe[j]:
                safe[j] = r[i]
    return safe

class GapController:
    def __init__(self, max_speed=2.5):
        self.max_speed = max_speed
        self.steer = 0.0

    def command(self, ranges, angle_min, increment):
        r = np.asarray(ranges, dtype=float)
        if r.size < 20 or increment <= 0:
            return 0.0, 0.0
        a = angle_min + np.arange(r.size) * increment
        r = np.nan_to_num(r, nan=0., posinf=15., neginf=0.)
        r = np.clip(r, 0., 15.)
        front = r[np.abs(a) < .18]
        if front.size == 0 or np.min(front) < .30:
            return 0.0, 0.0
        mask = np.abs(a) <= np.deg2rad(90)
        a, r = a[mask], r[mask]
        # Erosión angular: ensancha TODOS los obstáculos según el ancho del coche.
        safe = inflate_obstacles(r, increment)
        free = safe > 1.5
        edges = np.diff(np.r_[False, free, False].astype(int))
        gaps = list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))
        if not gaps:
            return 0., 0.
        lo, hi = max(gaps, key=lambda p: p[1]-p[0])
        score = safe[lo:hi] + .001 * np.cos(a[lo:hi])
        best = lo + int(np.argmax(score))
        target = float(a[best])
        # Dirección reactiva proporcional al ángulo del objetivo LiDAR.
        desired = target
        self.steer = float(np.clip(.5*self.steer + .5*desired, -.4189, .4189))
        speed = min(self.max_speed, 0.8 + 3.2*np.exp(-5*abs(self.steer)),
                    np.sqrt(2*2.0*max(0., float(np.min(front))-.25)))
        return self.steer, float(speed)

class LapCounter:
    """Cruce dirigido de una puerta, con distancia y tiempo mínimos antirrebote."""
    def __init__(self, x, y, yaw, min_distance=100., min_time=20., width=2.5):
        self.origin = np.array([x,y]); self.u = np.array([np.cos(yaw),np.sin(yaw)])
        self.v = np.array([-np.sin(yaw),np.cos(yaw)])
        self.min_distance, self.min_time, self.width = min_distance,min_time,width
        self.previous = None; self.started = None; self.last_t = None
        self.distance = 0.; self.times = []; self.invalid = False

    def update(self, x, y, t, collision=False):
        p = np.array([x,y], dtype=float)
        if collision:
            self.invalid = True
        if self.previous is None:
            self.previous = p; self.started = self.last_t = t
            return None
        step = np.linalg.norm(p-self.previous)
        if t < self.last_t or step > 5.:
            self.invalid = True
        self.last_t = t
        self.distance += step
        old = float((self.previous-self.origin) @ self.u)
        new = float((p-self.origin) @ self.u)
        crossed = old < 0 <= new
        if crossed:
            fraction = -old / (new-old)
            crossing = self.previous + fraction*(p-self.previous)
            lateral = abs(float((crossing-self.origin) @ self.v))
        else:
            lateral = float('inf')
        self.previous = p
        if (not self.invalid and crossed and lateral <= self.width
                and self.distance >= self.min_distance and t-self.started >= self.min_time):
            duration = t-self.started
            self.times.append(duration); self.started=t; self.distance=0.
            return duration
        return None
