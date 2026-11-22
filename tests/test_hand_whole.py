"""Whole-target camera dispatch. All network tests use a local fake robot."""
import math
import socketserver
import threading
import time
import unittest
from unittest.mock import patch

from test_control import OBSERVED_POSE, ready_status
from control import Controller, R3Client, load_config
from hand_control import HandControl
from hand_tracking.direct_core import Mapper, MultiMapper, FIELDS
from test_hand_control import frame
from serve_fake import Robot


class WholeTargetTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config());self.c.closed=True
        self.c.q=OBSERVED_POSE[:]
        self.h=HandControl(self.c,start_worker=False)
        self.now=time.monotonic();self.seq=0

    def tearDown(self):
        self.c.cancel.set()

    def packet(self,command,pose=None,axis=-1,axes=None):
        self.seq+=1
        p=dict(version=4,session='whole-test',sequence=self.seq,action=self.seq,
               command=command,sentMs=int(time.time()*1000),axis=axis)
        if pose is not None:p['pose']=pose+[1]
        if axes is not None:p['axes']=axes
        return self.h.packet(p,self.now)

    def start(self,real=False,multi=False):
        self.h.set_options(multi_axis=multi)
        if real:
            self.c.mode='monitor';self.c.client=object();self.c.armed=True
            self.c._synchronize_pose(OBSERVED_POSE);self.c.heartbeat();ready_status(self.c)
        self.h.enable(real=real,acknowledged=real)
        self.packet('STOP');return self.packet('CALIBRATE')

    def test_calibrate_default_mode_has_actual_goal_and_does_not_move(self):
        with patch.object(self.c,'move') as move:
            reply=self.start()
            self.h.tick(self.now+.025)
        move.assert_not_called()
        self.assertTrue(reply['whole_target']);self.assertFalse(reply['multi_axis'])
        self.assertEqual(list(reply['hand_reference'].values()),OBSERVED_POSE)

    def test_batch_sends_exact_goal_without_step_quantization(self):
        self.start(real=True)
        goal=OBSERVED_POSE[:];goal[0]+=4.237
        self.packet('TARGET',goal,0)
        with patch.object(self.c,'move') as move:
            self.h.tick(self.now+.019);move.assert_not_called()
            self.h.tick(self.now+.021)
        move.assert_called_once_with(goal,self.c.pose_revision,hand_target=True)
        self.assertGreater(goal[0]-OBSERVED_POSE[0],self.h.real_step)

    def test_busy_keeps_latest_six_axis_goal_and_stationary_hand_finishes_it(self):
        self.start(real=True,multi=True)
        self.c.busy=True;self.c.operation='move'
        with patch.object(self.c,'move') as move:
            for k in range(1,5):
                self.now+=.025;goal=OBSERVED_POSE[:];goal[0]+=k;goal[1]+=k*.3
                self.packet('TARGET',goal,axes=[0,1]);self.h.tick(self.now)
            move.assert_not_called()
            self.now+=.025;self.packet('TARGET',goal,axes=[])
            self.c.busy=False;self.h.tick(self.now)
        move.assert_called_once_with(goal,self.c.pose_revision,hand_target=True)
        self.c.q=goal[:]
        with patch.object(self.c,'move') as move:
            self.now+=.03;self.packet('TARGET',goal,axes=[]);self.h.tick(self.now)
        move.assert_not_called()

    def test_sequential_axis_changes_preserve_other_pending_angles(self):
        self.start(real=True)
        first=OBSERVED_POSE[:];first[0]+=3.125
        self.packet('TARGET',first,0)
        self.now+=.01
        second=OBSERVED_POSE[:];second[1]-=2.25
        self.packet('TARGET',second,1)
        expected=first[:];expected[1]=second[1]
        with patch.object(self.c,'move') as move:self.h.tick(self.now+.011)
        move.assert_called_once_with(expected,self.c.pose_revision,hand_target=True)

    def test_pause_and_watchdog_discard_unsent_goal(self):
        for watchdog in (False,True):
            with self.subTest(watchdog=watchdog):
                self.h=HandControl(self.c,start_worker=False);self.seq=0;self.start()
                goal=OBSERVED_POSE[:];goal[0]+=5;self.packet('TARGET',goal,0)
                with patch.object(self.c,'stop'),patch.object(self.c,'move') as move:
                    if watchdog:self.h.tick(self.now+.301)
                    else:self.h.pause()
                    self.h.tick(self.now+.32)
                move.assert_not_called();self.assertFalse(self.h.active)
                self.assertIsNone(self.h.target);self.assertIsNone(self.h.pending_since)

    def test_operator_limits_clamp_each_requested_channel(self):
        self.start(real=True,multi=True)
        self.c.motion_limits[0]=[OBSERVED_POSE[0]-1,OBSERVED_POSE[0]+1]
        self.c.motion_limits[1]=[OBSERVED_POSE[1]-1,OBSERVED_POSE[1]+1]
        goal=OBSERVED_POSE[:];goal[0]+=10;goal[1]-=10
        reply=self.packet('TARGET',goal,axes=[0,1])
        expected=OBSERVED_POSE[:];expected[0]+=1;expected[1]-=1
        with patch.object(self.c,'move') as move:self.h.tick(self.now+.021)
        move.assert_called_once_with(expected,self.c.pose_revision,hand_target=True)
        self.assertIn('blocked',reply['ikStatus'])

    def test_multi_requires_explicit_mode_and_modes_require_pause(self):
        self.start()
        with self.assertRaises(ValueError):self.h.set_options(multi_axis=True)
        with self.assertRaises(ValueError):self.packet('TARGET',OBSERVED_POSE,axes=[0,1])
        self.assertFalse(self.h.active)
        self.h.set_options(multi_axis=True)
        with self.assertRaises(ValueError):self.h.set_options(whole_target=False)
        self.h.set_options(whole_target=False,multi_axis=False)
        self.assertFalse(self.h.whole_target)

    def test_model_moves_multiple_joints_and_keeps_actual_feedback_separate(self):
        self.start(multi=True);goal=OBSERVED_POSE[:];goal[0]+=2;goal[1]-=2
        reply=self.packet('TARGET',goal,axes=[0,1])
        self.assertEqual(list(reply['pose'].values()),OBSERVED_POSE)
        self.assertEqual(list(reply['hand_reference'].values()),goal)
        self.h.tick(self.now+.02)
        self.assertGreater(self.c.q[0],OBSERVED_POSE[0]);self.assertLess(self.c.q[1],OBSERVED_POSE[1])
        self.assertEqual(self.c.q[2:],OBSERVED_POSE[2:])

    def test_full_tcp_sends_one_six_joint_mov_then_latest_goal(self):
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
        server.daemon_threads=True;server.q=OBSERVED_POSE[:];server.ready=True
        server.start_wait=True;server.exec_delay=.01;server.joint_speed=100
        server.commands=[];server.manual_step=720
        threading.Thread(target=server.serve_forever,daemon=True).start()
        self.c.config.update(host='127.0.0.1',port=server.server_address[1],timeout_s=.5,request_terminator='\n')
        client=R3Client(self.c.config);client.connect();self.c.client=client;self.c.mode='monitor'
        self.c._synchronize_pose(client.joints());self.c.heartbeat()
        done=threading.Event()
        def heartbeat():
            while not done.wait(.02):self.c.heartbeat()
        threading.Thread(target=heartbeat,daemon=True).start()
        try:
            self.c.arm(True)
            self.h.set_options(multi_axis=True);self.h.enable(real=True,acknowledged=True)
            self.packet('STOP');self.packet('CALIBRATE')
            goal=OBSERVED_POSE[:];goal[0]+=4.237;goal[1]-=2.125;goal[4]-=6.432
            self.packet('TARGET',goal,axes=[0,1,4]);self.h.tick(self.now+.021)
            deadline=time.monotonic()+3
            while self.c.busy and time.monotonic()<deadline:time.sleep(.005)
            self.assertFalse(self.c.busy,msg=str(self.c.events))
            for a,b in zip(server.q,goal):self.assertAlmostEqual(a,b,places=4)
            self.assertEqual(server.commands.count('EXECMov JPCDX'),1)
            self.assertEqual(sum(x.startswith('EXECJPCDX=') for x in server.commands),1)
            self.assertNotIn('SRVON',server.commands)
            self.assertFalse(any(x.startswith(('NEW','SAVE','LOAD','RUN','START','EXECH')) for x in server.commands))
            self.assertTrue(self.c.armed)
        finally:
            done.set();client.close();self.c.client=None;server.shutdown();server.server_close()


class MultiMappingTests(unittest.TestCase):
    def test_diagonal_translation_and_rotation_update_together(self):
        mapper=MultiMapper();mapper.calibrate(frame(),dict(zip(FIELDS,OBSERVED_POSE)),0)
        for k in range(1,31):
            t=k/60;f=frame();f['x']+=t*.18;f['y']-=t*.18
            f['q']=[0,0,math.sin(math.radians(t*35)/2),math.cos(math.radians(t*35)/2)]
            out,axes=mapper.update(f,t)
        self.assertEqual(axes,[0,1,5])
        for i in axes:self.assertGreater(out[i]-OBSERVED_POSE[i],1)
        for i in (2,3,4):self.assertEqual(out[i],OBSERVED_POSE[i])

    def test_high_gain_does_not_activate_stationary_jitter(self):
        mapper=MultiMapper(50);mapper.calibrate(frame(),dict(zip(FIELDS,OBSERVED_POSE)),0)
        for k in range(1,100):
            f=frame();f['x']+=.0003 if k%2 else -.0003
            out,axes=mapper.update(f,k/60)
            self.assertEqual(axes,[]);self.assertEqual(out[:6],OBSERVED_POSE)

    def test_single_mapper_switch_rebases_at_goal_instead_of_stale_encoders(self):
        mapper=Mapper();pose=dict(zip(FIELDS,OBSERVED_POSE));mapper.calibrate(frame(),pose,0)
        goal=OBSERVED_POSE[:];goal[0]+=4
        mapper.reconcile(dict(pose=pose,hand_reference=dict(zip(FIELDS,goal)),whole_target=True))
        for k in range(1,31):
            f=frame();f['y']-=k/60*.18;out,axis=mapper.update(f,k/60)
        self.assertEqual(axis,1);self.assertEqual(out[0],goal[0])
        self.assertGreater(out[1],OBSERVED_POSE[1])


if __name__=='__main__':unittest.main()
