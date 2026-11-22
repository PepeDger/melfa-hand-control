"""Controller override, exact TCP replies and idle-only application. No hardware."""
import time
import unittest
from control import Controller, R3Client, load_config
from diagnostics import parse_state
from test_control import FakeRobot, OBSERVED_POSE, READY_STATE, ready_status


class SpeedTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config()); self.c.closed=True

    def tearDown(self):
        self.c.cancel.set()
        if self.c.client: self.c.client.close()

    def prepare(self, wrong_readback=False):
        class Peer:
            def __init__(self): self.calls=[]; self.override=1
            def status(self):
                return dict(parse_state(READY_STATE),override=1 if wrong_readback else self.override)
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd.startswith('OVRD='): self.override=int(cmd[5:])
            def close(self): self.calls.append('close')
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'; self.c.armed=True
        ready_status(self.c); self.c.heartbeat(); self.c._synchronize_pose(OBSERVED_POSE)
        return peer

    def test_read_only_selection_sends_nothing_and_does_not_arm(self):
        peer=self.prepare(); self.c.armed=False
        self.c.set_speed(75)
        self.assertEqual(peer.calls,[])
        self.assertEqual(self.c.override_percent,75)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.speed_commands,('OVRD=75','EXECJOvrd 100','EXECAccel 1,1'))

    def test_armed_override_is_applied_and_confirmed_without_motion(self):
        peer=self.prepare()
        for percent in (1,50,100):
            self.c.set_speed(percent,self.c.pose_revision)
            self.assertEqual(self.c.snapshot()['speed_percent'],percent)
            self.assertEqual(self.c.controller_status['override'],percent)
        self.assertEqual(peer.calls,['OVRD=1','OVRD=50','OVRD=100'])
        self.assertTrue(self.c.armed)
        self.assertFalse(self.c.busy)

    def test_failed_readback_disarms_without_committing_selection(self):
        peer=self.prepare(wrong_readback=True)
        with self.assertRaisesRegex(ValueError,'OVRD=50'):
            self.c.set_speed(50,self.c.pose_revision)
        self.assertEqual(self.c.override_percent,1)
        self.assertFalse(self.c.armed)
        self.assertIn('STOP',peer.calls); self.assertIn('CNTLOFF',peer.calls)
        self.assertFalse(any('Mov' in cmd for cmd in peer.calls))

    def test_busy_invalid_and_stale_requests_do_not_write_override(self):
        peer=self.prepare()
        for value in (True,0,101,1.5,'50',None):
            with self.assertRaises(ValueError): self.c.set_speed(value,self.c.pose_revision)
        for operation in ('move','home','arming','speed'):
            self.c.busy=True; self.c.operation=operation
            with self.assertRaises(ValueError): self.c.set_speed(100,self.c.pose_revision)
        self.c.busy=False
        self.c.heartbeat_at=time.monotonic()-5
        with self.assertRaises(ValueError): self.c.set_speed(100,self.c.pose_revision)
        self.assertEqual(peer.calls,[])

    def test_three_digit_echo_waits_for_all_fragmented_digits(self):
        robot=FakeRobot([[b'QoK',(.17,b'1'),(.17,b'0'),(.17,b'0')],[b'QoK']])
        client=robot.client(); client.config.update(reply_terminator='',timeout_s=1.)
        self.assertEqual(client.command('OVRD=100'),'100')
        self.assertEqual(client.command('CNTLOFF'),'')
        client.close()

    def test_wrong_echo_is_rejected_and_not_retried(self):
        for reply in (b'QoK1\r\n',b'QoK101\r\n',b'QoK\r\n'):
            robot=FakeRobot([[reply]]); client=robot.client()
            with self.assertRaises(ValueError): client.command('OVRD=100')
            self.assertIsNone(client.sock)
            self.assertEqual(robot.commands,[b'1;1;OVRD=100'])


if __name__=='__main__': unittest.main()
