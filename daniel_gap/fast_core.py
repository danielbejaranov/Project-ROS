"""Reactive Follow-the-Gap candidate, LiDAR only; no map or odometry steering."""
import numpy as np
from daniel_gap.core import inflate_obstacles

class FastGapController:
    def __init__(self, max_speed=8., gain=.7, lateral_accel=3., brake_accel=3., smooth=31, alpha=.6):
        self.max_speed=float(max_speed); self.gain=float(gain)
        self.lateral_accel=float(lateral_accel); self.brake_accel=float(brake_accel)
        self.smooth=int(smooth); self.alpha=float(alpha); self.steer=0.
    def command(self,ranges,angle_min,increment):
        r=np.asarray(ranges,dtype=float)
        if r.size<20 or increment<=0: return 0.,0.
        a=angle_min+np.arange(r.size)*increment
        r=np.clip(np.nan_to_num(r,nan=0.,posinf=15.,neginf=0.),0.,15.)
        front=r[np.abs(a)<.18]
        if front.size==0 or np.min(front)<.30: return 0.,0.
        frontal=float(np.min(front))
        mask=np.abs(a)<=np.pi/2; a,r=a[mask],r[mask]
        safe=inflate_obstacles(r,increment)
        free=safe>1.5
        edges=np.diff(np.r_[False,free,False].astype(int))
        gaps=list(zip(np.where(edges==1)[0],np.where(edges==-1)[0]))
        if not gaps:return 0.,0.
        lo,hi=max(gaps,key=lambda p:p[1]-p[0])
        score=np.convolve(safe[lo:hi],np.ones(self.smooth)/self.smooth,mode='same')
        # Same width as gap even when gap is shorter than the convolution kernel.
        if score.size!=hi-lo:
            score=safe[lo:hi].copy()
        score=score+.02*np.cos(a[lo:hi])
        best=lo+int(np.argmax(score)); target=float(a[best])
        desired=np.clip(self.gain*target,-.4189,.4189)
        self.steer=float(np.clip((1-self.alpha)*self.steer+self.alpha*desired,-.4189,.4189))
        curvature=abs(np.tan(self.steer))/.3302
        speed=min(self.max_speed,np.sqrt(self.lateral_accel/max(curvature,.001)),
                  np.sqrt(2*self.brake_accel*max(0.,frontal-.50)))
        return self.steer,float(speed)
