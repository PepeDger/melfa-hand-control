"""Manual range configuration and bounded submission; no physical robot."""
import math
import time
import unittest
from unittest.mock import patch

from control import Controller, load_config
from test_control import OBSERVED_POSE, ready_status


class ManualRangeTests(unittest.TestCase):
    def setUp(self):
        config=load_config(); config['manual_step_deg']=20.
        self.c=Controller(config); self.c.closed=True

    def tearDown(self):
        self.c.cancel.set(); self.c.busy=False

    def prepare(self):
        self.c.mode='monitor'; self.c.armed=True; self.c.client=object()
        self.c.heartbeat_at=time.monotonic(); ready_status(self.c)
        self.c._synchronize_pose(OBSERVED_POSE)

    def test_twenty_degree_range_does_not_claim_commissioning(self):
        self.assertEqual(self.c.max_step,20)
        self.assertFalse(self.c.config['motion_commissioned'])
        self.assertFalse(self.c.config['home_motion_commissioned'])

    def test_positive_and_negative_twenty_degree_step_can_be_submitted(self):
        for delta in (-20.,20.):
            self.prepare(); target=OBSERVED_POSE[:]; target[0]+=delta
            with patch('control.threading.Thread') as worker:
                self.c.move(target,self.c.pose_revision)
                worker.return_value.start.assert_called_once()
            self.assertEqual(self.c.target,target)
            self.assertEqual(self.c.target[1:],OBSERVED_POSE[1:])
            self.c.busy=False

    def test_larger_target_is_rejected_and_multiple_axes_are_allowed(self):
        self.prepare()
        for delta in (20.00001,-20.00001):
            target=OBSERVED_POSE[:]; target[0]+=delta
            with self.assertRaises(ValueError): self.c.move(target,self.c.pose_revision)
        target=OBSERVED_POSE[:]; target[0]+=1; target[1]+=1
        with patch('control.threading.Thread'):
            self.c.move(target,self.c.pose_revision)
        self.assertEqual(self.c.target,target)
        self.c.busy=False

    def test_invalid_configured_ranges_are_rejected(self):
        for value in (True,'90',None,0,-1,720.001,math.nan,math.inf):
            config=load_config(); config['manual_step_deg']=value
            with self.subTest(value=value), self.assertRaises(ValueError): Controller(config)

    def test_factory_joint_limits_still_apply(self):
        self.prepare()
        pose=OBSERVED_POSE[:]; pose[0]=169.5
        self.c._synchronize_pose(pose)
        pose[0]=171
        with self.assertRaises(ValueError): self.c.move(pose,self.c.pose_revision)

    def test_chosen_speed_changes_joint_cap_but_keeps_override_and_ramp(self):
        self.assertEqual(self.c.speed_commands,('OVRD=1','EXECJOvrd 100','EXECAccel 1,1'))
        self.assertEqual(self.c.snapshot()['speed_factor'],10)
        self.assertTrue(self.c.snapshot()['home_motion_enabled'])
        self.assertEqual(self.c.snapshot()['home_motion_mode'],'coordinated')

    def test_speed_options_and_invalid_values(self):
        for speed in (10,20,50,100):
            config=load_config(); config['joint_speed_percent']=speed
            c=Controller(config); c.closed=True
            self.assertEqual(c.speed_commands[1],f'EXECJOvrd {speed}')
        for speed in (True,'100',None,0,101,math.nan,math.inf):
            config=load_config(); config['joint_speed_percent']=speed
            with self.subTest(speed=speed),self.assertRaises(ValueError): Controller(config)

    def test_home_switch_must_be_boolean(self):
        config=load_config(); config['home_motion_enabled']='true'
        with self.assertRaises(ValueError): Controller(config)

    def test_ninety_degree_range_accepts_both_directions_but_not_more(self):
        self.assertEqual(load_config()['manual_step_deg'],720)
        self.c.manual_step=90.
        for delta in (-90.,90.):
            self.prepare(); target=OBSERVED_POSE[:]; target[0]+=delta
            with patch('control.threading.Thread') as worker:
                self.c.move(target,self.c.pose_revision)
                worker.return_value.start.assert_called_once()
            self.c.busy=False
        self.prepare(); target=OBSERVED_POSE[:]; target[0]+=90.01
        with self.assertRaises(ValueError): self.c.move(target,self.c.pose_revision)

    def test_factory_mode_accepts_full_wrist_rotation_and_keeps_factory_bounds(self):
        self.c.manual_step=720.
        self.prepare(); pose=OBSERVED_POSE[:]; pose[5]=-360.
        self.c._synchronize_pose(pose)
        target=pose[:]; target[5]=360.
        with patch('control.threading.Thread'):
            self.c.move(target,self.c.pose_revision)
        self.assertTrue(self.c.snapshot()['full_joint_ranges'])
        self.c.busy=False; target[5]=360.01
        with self.assertRaises(ValueError): self.c.move(target,self.c.pose_revision)


if __name__=='__main__': unittest.main()
