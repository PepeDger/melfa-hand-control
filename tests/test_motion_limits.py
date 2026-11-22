"""Persistent operator limits, independent telemetry and all motion sources."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch, Mock
from control import Controller, load_config, validate_motion_limits, MotionBlocked
from hand_control import HandControl
from test_control import OBSERVED_POSE, ready_status


class MotionLimitsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/'motion_limits.json'
        self.config=load_config()
        self.c=Controller(self.config, limits_path=self.path);self.c.closed=True

    def tearDown(self):
        self.c.cancel.set();self.temp.cleanup()

    def limits(self):
        bounds=copy.deepcopy(self.config['joint_limits']);bounds[0]=[-10,10]
        return bounds

    def test_all_six_limits_persist_and_reloading_keeps_fixed_home(self):
        bounds=self.limits();self.c.set_motion_limits(bounds)
        self.assertEqual(json.loads(self.path.read_text())['limits'],bounds)
        restored=Controller(self.config,limits_path=self.path);restored.closed=True
        self.assertEqual(restored.snapshot()['limits'],bounds)
        self.assertEqual(restored.home_pose,self.c.home_pose)
        self.assertEqual(restored.snapshot()['physical_limits'],self.config['joint_limits'])

    def test_bad_ranges_are_rejected_without_partial_changes(self):
        self.c.set_motion_limits(self.limits());before=self.path.read_bytes()
        invalids=[[],None,[[0,1]]*5]
        for pair in ([10,10],[11,10],[-171,10],[0,171],[False,10],[0,'10'],[0,None],[math.nan,10],[0,math.inf]):
            bounds=self.limits();bounds[0]=pair;invalids.append(bounds)
        for bounds in invalids:
            with self.subTest(bounds=bounds),self.assertRaises(ValueError):self.c.set_motion_limits(bounds)
            self.assertEqual(self.path.read_bytes(),before)
            self.assertEqual(self.c.motion_limits,self.limits())

    def test_failed_save_does_not_change_active_limits(self):
        self.c.set_motion_limits(self.limits());before=self.path.read_bytes()
        with patch('control.os.replace',side_effect=OSError('disk error')):
            with self.assertRaises(OSError):self.c.set_motion_limits(self.config['joint_limits'])
        self.assertEqual(self.c.motion_limits,self.limits());self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(list(Path(self.temp.name).glob('*.tmp')),[])

    def test_armed_and_busy_limits_cannot_change_and_no_robot_commands_are_sent(self):
        self.c.client=Mock();initial=copy.deepcopy(self.c.motion_limits)
        for armed,busy in ((True,False),(False,True)):
            self.c.armed=armed;self.c.busy=busy
            with self.assertRaises(ValueError):self.c.set_motion_limits(self.limits())
        self.assertEqual(self.c.motion_limits,initial);self.assertFalse(self.path.exists())
        self.c.client.command.assert_not_called()

    def test_actual_pose_outside_limits_can_be_read_but_motion_is_blocked(self):
        bounds=self.limits();bounds[2]=[-20,100]
        self.c.set_motion_limits(bounds)
        self.c._synchronize_pose(OBSERVED_POSE)
        self.assertEqual(self.c.q,OBSERVED_POSE)
        self.assertTrue(self.c.snapshot()['pose_synced'])
        self.assertIn('J3',self.c.snapshot()['limit_violations'][0])
        with self.assertRaises(MotionBlocked):self.c.move([0]*6,self.c.pose_revision)
        self.assertEqual(self.c.q,OBSERVED_POSE)

    def test_home_outside_limits_is_refused_without_changing_saved_home(self):
        bounds=self.limits();bounds[2]=[-20,100];self.c.set_motion_limits(bounds)
        home=self.c.home_pose
        with patch('control.threading.Thread') as worker:
            with self.assertRaises(ValueError):self.c.home()
            worker.assert_not_called()
        self.assertFalse(self.c.snapshot()['home_within_limits'])
        self.assertEqual(self.c.home_pose,home)
        restored=Controller(self.config,limits_path=self.path);restored.closed=True
        self.assertEqual(restored.home_pose,home)

    def test_manual_and_direct_real_dispatch_cannot_bypass_limits(self):
        self.c.set_motion_limits(self.limits())
        target=[11,0,0,0,0,0]
        with patch('control.threading.Thread') as worker:
            with self.assertRaises(ValueError):self.c.move(target)
            worker.assert_not_called()
        self.c.client=Mock()
        with self.assertRaises(ValueError):self.c._execute_real_step(target)
        self.c.client.command.assert_not_called()

    def test_hand_reaches_limit_without_overshoot_and_can_move_back(self):
        bounds=self.limits();bounds[0]=[-1,1];self.c.set_motion_limits(bounds)
        hand=HandControl(self.c,start_worker=False,whole_target=False);hand.enable()
        now=time.monotonic();sequence=0
        def packet(command,pose=None):
            nonlocal sequence
            sequence+=1
            p=dict(version=4,session='test',sequence=sequence,action=sequence,command=command,
                   sentMs=int(time.time()*1000),axis=0)
            if pose is not None:p['pose']=pose
            return hand.packet(p,now)
        packet('STOP');packet('CALIBRATE')
        for k in range(100):
            now+=.02;reply=packet('TARGET',[30,0,0,0,0,0,1]);hand.tick(now)
            self.assertLessEqual(self.c.q[0],1)
            self.assertEqual(self.c.q[1:],[0]*5)
        self.assertAlmostEqual(self.c.q[0],1)
        self.assertIn('blocked',reply['ikStatus'])
        for k in range(20):now+=.02;packet('TARGET',[-.5,0,0,0,0,0,1]);hand.tick(now)
        self.assertLess(self.c.q[0],1)

    def test_servo_off_remains_available_outside_operator_limits(self):
        bounds=self.limits();bounds[2]=[-20,100];self.c.set_motion_limits(bounds)
        self.c._synchronize_pose(OBSERVED_POSE);ready_status(self.c)
        self.assertTrue(any('J3' in x for x in self.c._status_blockers()))
        self.c.operation='servo'
        self.assertFalse(any('J3' in x for x in self.c._status_blockers(require_servo=False)))

    def test_corrupted_saved_limits_do_not_silently_revert_to_unrestricted(self):
        self.path.write_text('{"version":1,"limits":[]}')
        with self.assertRaises(ValueError):Controller(self.config,limits_path=self.path)

    def test_offline_model_starts_inside_saved_limits_and_connection_still_uses_actual_joints(self):
        bounds=self.limits();bounds[1]=[-20,-4];bounds[2]=[130,159]
        self.c.set_motion_limits(bounds)
        restored=Controller(self.config,limits_path=self.path);restored.closed=True
        self.assertEqual(restored.mode,'simulation')
        self.assertEqual(restored.q,list(restored.home_pose))
        self.assertFalse(restored._limit_violations())
        actual=OBSERVED_POSE[:];actual[0]=20
        restored._synchronize_pose(actual)
        self.assertEqual(restored.q,actual)


if __name__=='__main__':unittest.main()
