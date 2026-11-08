import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from motion_host import VelocityBridge, body_velocity
from chassis_serial import decode
import struct

class Client:
    def __init__(self): self.sent=[]
    def send(self, *args): self.sent.append(args)

class Tests(unittest.TestCase):
    def test_stale_ros_does_not_renew(self):
        c=Client();now=[1.];b=VelocityBridge(c,lambda:now[0])
        b.on_twist(.1,0);b.poll();self.assertEqual(c.sent[-1][0],0x10)
        now[0]=1.1;b.poll();self.assertLessEqual(struct.unpack('<ffH',c.sent[-1][1])[2],100)
        now[0]=1.16;b.poll();self.assertEqual(c.sent[-1][0],0x12)
        n=len(c.sent);b.on_twist(.1,0);b.poll();self.assertEqual(len(c.sent),n)
    def test_body_feedback(self):
        self.assertAlmostEqual(body_velocity([4,4,4,4],.0425,.228)["vx"],.17)
        self.assertAlmostEqual(body_velocity([-2,-2,2,2],.0425,.228)["wz"],.17/.228)
        self.assertRaises(ValueError,body_velocity,[0,0,0,float("nan")],.0425,.228)
    def test_invalid(self):
        b=VelocityBridge(Client());self.assertRaises(ValueError,b.on_twist,float('nan'),0)
    def test_control_decode(self):
        d=decode({'id':0x24,'payload':struct.pack('<8I8f4i',*([0]*20))})
        self.assertEqual(d['duty'],(0,0,0,0));self.assertEqual(d['state'],0)

unittest.main()
