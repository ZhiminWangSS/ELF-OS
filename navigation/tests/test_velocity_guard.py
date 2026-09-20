import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from velocity_guard import VelocityGuard
class GuardTests(unittest.TestCase):
    def test_requires_fresh_command_and_localization(self):
        g=VelocityGuard();g.receive([.1,0,0,0,0,.2],10)
        self.assertEqual(g.output(10.1),(0.,0.));g.healthy(True,10)
        self.assertEqual(g.output(10.1),(.1,.2));self.assertEqual(g.output(10.26),(0.,0.))
        g.healthy(True,11);self.assertEqual(g.output(11),(0.,0.))
    def test_clamps_limits(self):
        g=VelocityGuard();g.healthy(True,1);g.receive([10,0,0,0,0,-5],1)
        self.assertEqual(g.output(1),(.2,-.35))
    def test_health_timeout_is_separate_from_command_timeout(self):
        g=VelocityGuard(timeout=.25,health_timeout=.5);g.healthy(True,0);g.receive([.1,0,0,0,0,0],.1)
        self.assertEqual(g.output(.2),(.1,0.));self.assertEqual(g.output(.61),(0.,0.))
    def test_cancel_zero_is_immediate(self):
        g=VelocityGuard();g.healthy(True,1);g.receive([.2,0,0,0,0,0],1);g.receive([0]*6,1.01)
        self.assertEqual(g.output(1.01),(0.,0.))
    def test_invalid_latches(self):
        for values in [[float('nan'),0,0,0,0,0],[.1,.01,0,0,0,0],[-.1,0,0,0,0,0]]:
            g=VelocityGuard();g.healthy(True,1);g.receive(values,1);g.receive([.1,0,0,0,0,0],1)
            self.assertTrue(g.latched);self.assertEqual(g.output(1),(0.,0.))
if __name__=='__main__':unittest.main()
