"""Regresión numérica de la optimización usada en ROS; sin cambiar el algoritmo."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from daniel_gap.core import inflate_obstacles
rng=np.random.default_rng(17)
for _ in range(100):
    r=rng.uniform(.01,15,720);inc=4.7/1079;reference=r.copy()
    for i in range(len(r)):
        half=int(np.arctan2(.4,max(r[i],.01))/inc)
        lo,hi=max(0,i-half),min(len(r),i+half+1)
        reference[lo:hi]=np.minimum(reference[lo:hi],r[i])
    np.testing.assert_array_equal(inflate_obstacles(r,inc),reference)
print('OK: 100 escaneos, igualdad exacta con la erosión NumPy original.')
