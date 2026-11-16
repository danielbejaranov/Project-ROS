import unittest
import numpy as np
from daniel_gap.core import GapController,LapCounter
class CoreTests(unittest.TestCase):
    def test_bad_scans_stop(self):
        for value in [0.,float('nan'),float('-inf')]:
            self.assertEqual(GapController().command([value]*1080,-2.35,4.7/1079),(0.,0.))
    def test_open_corridor(self):
        steering,speed=GapController().command([10.]*1080,-2.35,4.7/1079)
        self.assertLess(abs(steering),.01); self.assertGreater(speed,0.)
    def test_directed_gate_and_collision(self):
        c=LapCounter(0,0,0,min_distance=4,min_time=2,width=2)
        for x,y,t in [(0,0,0),(3,0,1),(3,3,2),(-2,3,3),(-2,0,4),(1,0,5)]:
            c.update(x,y,t)
        self.assertEqual(c.times,[5])
        c.update(-.2,0,5.1); c.update(.2,0,5.2)
        self.assertEqual(len(c.times),1)
        c.update(-2,0,8,True); c.update(2,0,15)
        self.assertEqual(len(c.times),1); self.assertTrue(c.invalid)
    def test_gate_rejects_wrong_direction_and_wrong_lane(self):
        c=LapCounter(0,0,0,min_distance=0,min_time=0,width=1)
        c.update(1,0,0); c.update(-1,0,1)
        self.assertEqual(c.times,[])
        c.update(-1,2,2); c.update(1,2,3)
        self.assertEqual(c.times,[])
    def test_reset_invalidates(self):
        c=LapCounter(0,0,0); c.update(0,0,1); c.update(10,0,2)
        self.assertTrue(c.invalid)
if __name__=='__main__': unittest.main()
