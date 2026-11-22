"""One coordinated target for 2..6 axes; reject stale starting positions."""
import unittest
from unittest.mock import patch
from control import Controller, MotionBlocked, load_config
from diagnostics import parse_state
from test_control import OBSERVED_POSE, READY_STATE, ready_status

class MultiAxisTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config());self.c.closed=True
        class Peer:
            def __init__(self): self.q=OBSERVED_POSE[:];self.calls=[];self.pending=None;self.reads=0;self.drift=0
            def joints(self):
                self.reads+=1
                q=self.q[:]
                if self.reads==self.drift: q[3]+=.02
                return q
            def status(self): return parse_state(READY_STATE)
            def close(self): pass
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd.startswith('EXECJPCDX='):
                    self.pending=[float(v) for v in cmd.split('(',1)[1][:-1].split(',')]
                if cmd=='EXECMov JPCDX': self.q=self.pending[:]
        self.peer=Peer();self.c.client=self.peer;self.c.mode='monitor';self.c.armed=True
        self.c._synchronize_pose(OBSERVED_POSE);self.c.heartbeat();ready_status(self.c)
        self.target=[a+b for a,b in zip(OBSERVED_POSE,[1,2,-3,4,-5,6])]

    def tearDown(self): self.c.cancel.set()

    def test_submission_accepts_two_and_six_axes_but_not_noop(self):
        for axes in (2,6):
            target=OBSERVED_POSE[:]
            target[:axes]=self.target[:axes]
            with patch('control.threading.Thread') as worker:
                self.c.move(target,self.c.pose_revision)
                worker.return_value.start.assert_called_once()
            self.assertEqual(self.c.target,target);self.c.busy=False
        with self.assertRaises(ValueError): self.c.move(OBSERVED_POSE,self.c.pose_revision)

    def test_six_axes_are_one_mov_and_feedback_confirms_full_pose(self):
        with patch.object(self.c.cancel,'wait',side_effect=lambda _:self.c.cancel.is_set()):
            self.assertTrue(self.c._execute_real_step(self.target))
        for actual,target in zip(self.c.q,self.target): self.assertAlmostEqual(actual,target,places=5)
        self.assertEqual(self.peer.calls.count('EXECMov JPCDX'),1)
        self.assertEqual(sum(cmd.startswith('EXECJPCDX=') for cmd in self.peer.calls),1)
        self.assertIn('EXECAccel 1,1',self.peer.calls)

    def test_displacement_before_or_during_setup_prevents_mov(self):
        for read in (1,2):
            with self.subTest(read=read):
                self.peer.reads=0;self.peer.calls=[];self.peer.drift=read
                with self.assertRaises(MotionBlocked): self.c._execute_real_step(self.target)
                self.assertNotIn('EXECMov JPCDX',self.peer.calls)
                if read==1: self.assertEqual(self.peer.calls,[])

if __name__=='__main__': unittest.main()
