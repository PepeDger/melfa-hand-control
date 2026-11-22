"""Explicit servo commands and telemetry checks; no physical robot."""
import time
import unittest
from control import Controller, load_config
from diagnostics import parse_state
from test_control import OBSERVED_POSE, READY_STATE


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config()); self.c.closed=True
        class Peer:
            def __init__(self): self.on=False; self.owned=False; self.calls=[]; self.ignore=False; self.error=0
            def status(self):
                return dict(parse_state(READY_STATE),servo_on=self.on,operation_enabled=self.owned,error=self.error)
            def joints(self): return OBSERVED_POSE[:]
            def close(self): pass
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd=='CNTLON': self.owned=True
                if cmd=='CNTLOFF': self.owned=False
                if cmd in ('SRVON','SRVOFF') and not self.ignore: self.on=cmd=='SRVON'
        self.peer=Peer();self.c.client=self.peer;self.c.mode='monitor'
        self.c._synchronize_pose(OBSERVED_POSE);self.c.heartbeat()

    def tearDown(self): self.c.cancel.set()

    def test_remote_on_from_servo_off_and_off_release_rights_without_motion(self):
        self.c.set_servo(True,True)
        self.assertEqual(self.peer.calls,['CNTLON','SRVON','CNTLOFF'])
        self.assertTrue(self.c.controller_status['servo_on']);self.assertFalse(self.c.armed)
        self.c.set_servo(False,True)
        self.assertEqual(self.peer.calls[-3:],['CNTLON','SRVOFF','CNTLOFF'])
        self.assertFalse(self.c.controller_status['servo_on']);self.assertFalse(self.c.armed)
        self.assertFalse(any('Mov' in cmd or cmd in ('START','RUN') for cmd in self.peer.calls))

    def test_off_from_prepared_session_disarms(self):
        self.peer.on=self.peer.owned=True;self.c.armed=True
        self.c.set_servo(False,True)
        self.assertEqual(self.peer.calls,['SRVOFF','CNTLOFF'])
        self.assertFalse(self.c.armed);self.assertFalse(self.c.operator_session)

    def test_no_command_for_already_matching_state(self):
        self.c.set_servo(False,True)
        self.assertEqual(self.peer.calls,[])

    def test_busy_missing_ack_and_alarm_prevent_power_on(self):
        with self.assertRaises(ValueError): self.c.set_servo(True,False)
        self.c.busy=True
        with self.assertRaises(ValueError): self.c.set_servo(True,True)
        self.c.busy=False;self.peer.error=123
        with self.assertRaises(ValueError): self.c.set_servo(True,True)
        self.assertNotIn('SRVON',self.peer.calls)

    def test_ack_without_feedback_does_not_count_as_servo_on(self):
        self.peer.ignore=True
        # Lose heartbeat immediately after ACK so the unconfirmed transition
        # aborts without waiting for the five-second confirmation deadline.
        original=self.peer.command
        def command(cmd):
            original(cmd)
            if cmd=='SRVON': self.c.heartbeat_at=time.monotonic()-2
        self.peer.command=command
        with self.assertRaises(ValueError): self.c.set_servo(True,True)
        self.assertFalse(self.c.armed);self.assertEqual(self.peer.calls[-1],'CNTLOFF')
        self.assertEqual(self.c.mode,'fault')


if __name__=='__main__': unittest.main()
