"""Operational checks using observed and synthetic frames; no robot motion."""
import time
import unittest
from unittest.mock import patch
from test_control import FakeRobot, OBSERVED_STATE, READY_STATE, OBSERVED_POSE, ready_status
from control import Controller, load_config
from diagnostics import parse_dstate, parse_state, movement_blockers


class DiagnosticTests(unittest.TestCase):
    def setUp(self):
        config=load_config(); config.update(manual_step_deg=.1,joint_speed_percent=10,home_motion_enabled=False)
        self.c = Controller(config)
        self.c.closed = True

    def tearDown(self):
        self.c.cancel.set()
        if self.c.client: self.c.client.close()

    def test_observed_state_matches_toolbox_servo_off(self):
        state = parse_state(OBSERVED_STATE)
        self.assertEqual(state['program'], 'MAIN.MB5')
        self.assertEqual(state['override'], 100)
        self.assertTrue(state['automatic'])
        self.assertFalse(state['servo_on'])
        self.assertFalse(state['running'])
        self.assertFalse(state['program_running'])
        self.assertEqual(state['error'], 0)
        self.assertEqual(len(movement_blockers(state)), 2)
        self.assertEqual(parse_dstate('01120;1;0')['error'], 0)

    def test_fragmented_status_frames_wait_for_complete_mechanism_field(self):
        robot = FakeRobot([[b'QoK'+OBSERVED_STATE[:-1].encode(),(.2,OBSERVED_STATE[-1:].encode())],
                           [b'QoK01120;',(.2,b'1;0')]])
        client = robot.client()
        client.config.update(reply_terminator='',timeout_s=.8)
        self.c.client = client
        state = client.status()
        self.assertFalse(state['servo_on'])
        self.assertEqual(robot.commands,[b'1;1;STATE',b'1;1;DSTATE'])

    def test_bad_or_truncated_status_is_rejected(self):
        for value in (OBSERVED_STATE[:-1], OBSERVED_STATE+';0', OBSERVED_STATE.replace('01120','XYZ'),
                      OBSERVED_STATE.replace(';100;',';101;'), OBSERVED_STATE.replace(';REP;',';OTHER;')):
            with self.subTest(value=value), self.assertRaises(ValueError): parse_state(value)
        for value in ('01120;1;', '01120;1;10', '01120;1;0;garbage', 'GG120;1;0'):
            with self.subTest(value=value), self.assertRaises(ValueError): parse_dstate(value)

    def test_each_unsafe_flag_blocks_motion(self):
        state = parse_state(READY_STATE)
        self.assertEqual(movement_blockers(state), [])
        for name, value in dict(servo_on=False, automatic=False, operation_enabled=False,
                                machine_lock=True, emergency_stop=True, error=1234,
                                running=True, program_running=True, editing=True,
                                waiting=True, pseudo_input=True, teach_running=True).items():
            with self.subTest(flag=name):
                changed = dict(state, **{name:value})
                self.assertTrue(movement_blockers(changed))

    def test_unknown_or_stale_status_prevents_move_without_sending(self):
        class Peer:
            def close(self): pass
            def command(self,cmd): raise AssertionError('No command should be sent')
        self.c.client=Peer()
        self.c.mode='monitor'
        self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        self.c._synchronize_pose(OBSERVED_POSE)
        goal=OBSERVED_POSE[:]; goal[0]+=.1
        with self.assertRaisesRegex(ValueError,'ещё не прочитано'):
            self.c.move(goal,self.c.pose_revision)
        ready_status(self.c)
        self.c.status_at-=2
        with self.assertRaisesRegex(ValueError,'устарело'):
            self.c.move(goal,self.c.pose_revision)

    def test_read_only_stop_and_disconnect_never_send_stop(self):
        class Peer:
            def __init__(self): self.closed=False
            def close(self): self.closed=True
            def command(self,cmd): raise AssertionError('Read-only must not interrupt programs')
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'
        self.c.stop()
        self.c.disconnect()
        self.assertTrue(peer.closed)
        self.assertEqual(self.c.mode,'simulation')

    def test_servo_off_prevents_cntlon_and_any_setup_command(self):
        class Peer:
            def status(self): return parse_state(OBSERVED_STATE)
            def command(self,cmd): raise AssertionError('No write before checks')
            def close(self): pass
        self.c.client=Peer(); self.c.mode='monitor'
        self.c.feedback_at=time.monotonic()
        self.c.config.update(motion_commissioned=True,controller_firmware='test-only')
        with self.assertRaisesRegex(ValueError,'Серво выключено'): self.c.arm(True)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.mode,'monitor')
        self.assertIsNotNone(self.c.client)

    def test_initial_motion_is_available_without_fabricating_firmware_or_commissioning(self):
        self.assertEqual(self.c.config['controller_firmware'],'')
        self.assertFalse(self.c.config['motion_commissioned'])
        self.assertTrue(self.c.physical_enabled)
        self.assertEqual(self.c.max_step,.1)
        self.assertTrue(self.c.snapshot()['initial_control'])

    def test_trial_mode_rejects_a_larger_step_and_home_without_sending(self):
        class Peer:
            def command(self,cmd): raise AssertionError('No motion for an invalid request')
            def close(self): pass
        self.c.client=Peer(); self.c.mode='monitor'; self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        ready_status(self.c)
        self.c._synchronize_pose(OBSERVED_POSE)
        target=OBSERVED_POSE[:]; target[0]+=.2
        with self.assertRaises(ValueError): self.c.move(target,self.c.pose_revision)
        self.c.config['home_motion_commissioned']=True
        with self.assertRaisesRegex(ValueError,'отключён'): self.c.home(self.c.pose_revision)

    def test_preparation_allows_heartbeat_and_releases_rights_when_cancelled(self):
        class Peer:
            def __init__(self,controller): self.controller=controller; self.calls=[]
            def status(self): return parse_state(READY_STATE)
            def command(self,cmd):
                self.calls.append(cmd)
                self.controller.heartbeat()
                if cmd=='CNTLON': self.controller.cancel.set()
            def close(self): pass
        peer=Peer(self.c); self.c.client=peer; self.c.mode='monitor'
        self.c.feedback_at=time.monotonic()
        with self.assertRaisesRegex(ValueError,'отменена'): self.c.arm(True)
        self.assertEqual(peer.calls,['CNTLON','CNTLOFF'])
        self.assertFalse(self.c.armed)
        self.assertFalse(self.c.busy)
        self.assertEqual(self.c.mode,'monitor')

    def test_initial_preparation_sends_no_motion_or_servo_on(self):
        class Peer:
            def __init__(self): self.calls=[]
            def status(self): return parse_state(READY_STATE)
            def joints(self): return OBSERVED_POSE[:]
            def command(self,cmd): self.calls.append(cmd)
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'
        self.c.feedback_at=self.c.heartbeat_at=time.monotonic()
        self.c.arm(True)
        self.assertTrue(self.c.armed)
        self.assertEqual(peer.calls,['CNTLON','OVRD=1','EXECJOvrd 10','EXECAccel 1,1'])
        self.assertEqual(self.c.q,self.c.target)
        self.assertEqual(self.c.max_step,.1)

    def test_changed_state_immediately_before_move_prevents_execmov(self):
        class Peer:
            def __init__(self): self.calls=[]; self.reads=0
            def joints(self): return OBSERVED_POSE[:]
            def status(self):
                self.reads+=1
                return parse_state(READY_STATE if self.reads==1 else OBSERVED_STATE)
            def command(self,cmd): self.calls.append(cmd)
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'; self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        self.c._synchronize_pose(OBSERVED_POSE)
        target=OBSERVED_POSE[:]; target[0]+=.1
        with self.assertRaisesRegex(ValueError,'Серво выключено'):
            self.c._execute_real_step(target)
        self.assertNotIn('EXECMov JPCDX',peer.calls)

    def test_speed_limit_must_be_read_back_before_motion(self):
        class Peer:
            def __init__(self): self.calls=[]
            def joints(self): return OBSERVED_POSE[:]
            def status(self): return dict(parse_state(READY_STATE),override=100)
            def command(self,cmd): self.calls.append(cmd)
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'; self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        self.c._synchronize_pose(OBSERVED_POSE)
        target=OBSERVED_POSE[:]; target[0]+=.1
        with self.assertRaisesRegex(ValueError,'OVRD=1'): self.c._execute_real_step(target)
        self.assertNotIn('EXECMov JPCDX',peer.calls)

    def test_diagnostic_operation_does_not_issue_write_commands(self):
        class Peer:
            def status(self): return parse_state(OBSERVED_STATE)
            def command(self,cmd): raise AssertionError('Diagnostics are read-only')
            def close(self): pass
        self.c.client=Peer(); self.c.mode='monitor'
        self.c.diagnose()
        deadline=time.monotonic()+1
        while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
        self.assertFalse(self.c.busy)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.controller_status['program'],'MAIN.MB5')


if __name__=='__main__': unittest.main()
