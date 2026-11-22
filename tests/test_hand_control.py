"""Camera mapping and bounded physical dispatch, never contacts lab equipment."""
import math
import base64
import socketserver
import threading
import time
import unittest
from unittest.mock import patch
from control import Controller, R3Client, load_config
from hand_control import HandControl
from hand_link import Link
from hand_tracking.direct_core import Mapper, FIELDS
from test_control import OBSERVED_POSE, ready_status
from serve_fake import Robot


def frame():
    return dict(x=.5, y=.5, size=.15, q=[0,0,0,1], grip=1, gesture='MOVE')


class HandTests(unittest.TestCase):
    def setUp(self):
        self.c = Controller(load_config())
        self.c.closed = True
        self.h = HandControl(self.c, start_worker=False, whole_target=False)
        self.now = time.monotonic()
        self.seq = 0
        self.action = 0
        self.mode = None

    def tearDown(self):
        self.c.cancel.set()

    def packet(self, command, pose=None, axis=-1, **extras):
        self.seq += 1
        if command != self.mode:
            self.action += 1
        self.mode = command
        p = dict(version=4, session='camera-test', sequence=self.seq, action=self.action,
                 command=command, sentMs=int(time.time()*1000), ageMs=0, axis=axis)
        if pose is not None:
            p['pose'] = pose
        p.update(extras)
        return self.h.packet(p, self.now)

    def start(self, grip=False, real=False):
        self.h.enable(real, grip, real)
        self.assertEqual(self.packet('STOP')['command'], 'STOP')
        self.assertEqual(self.packet('CALIBRATE')['command'], 'CALIBRATE')

    def make_real(self):
        self.c.mode='monitor'
        self.c.client=object()
        self.c.armed=True
        self.c._synchronize_pose(OBSERVED_POSE)
        self.c.heartbeat()
        ready_status(self.c)

    def test_six_camera_movements_choose_only_their_axis(self):
        pose=dict(zip(FIELDS, OBSERVED_POSE))
        for joint in range(6):
            with self.subTest(joint=joint):
                m=Mapper();m.calibrate(frame(),pose,0)
                for k in range(1,31):
                    f=frame();t=k/60
                    if joint==0: f['x']+=t*.18
                    elif joint==1: f['y']-=t*.18
                    elif joint==2: f['size']*=math.exp(t*.5)
                    else:
                        r=math.radians(t*35)/2
                        f['q'][{3:1,4:0,5:2}[joint]]=math.sin(r)
                        f['q'][3]=math.cos(r)
                    out,axis=m.update(f,t)
                self.assertEqual(axis,joint)
                # J3 starts near its limit, so only test nonzero clipped travel.
                self.assertGreater(abs(out[joint]-OBSERVED_POSE[joint]),1)
                for i in range(6):
                    if i!=joint: self.assertEqual(out[i],OBSERVED_POSE[i])

    def test_calibration_anchors_actual_pose_without_motion(self):
        self.c.q=OBSERVED_POSE[:]
        self.start()
        self.assertEqual(list(self.h._reply(dict(session='x',sequence=0,action=0),'CALIBRATE')['pose'].values()),OBSERVED_POSE)
        self.assertEqual(self.c.q,OBSERVED_POSE)
        self.assertIsNone(self.h.target)

    def test_stop_handshake_required_and_repeated_calibration_is_idempotent(self):
        self.h.enable()
        with self.assertRaises(ValueError): self.packet('CALIBRATE')
        self.packet('STOP');self.packet('CALIBRATE')
        self.now+=.05
        self.packet('CALIBRATE')
        self.assertTrue(self.h.active)
        self.assertLess(self.h.last_packet,self.now)

    def test_model_acceleration_single_axis_and_factory_clamp(self):
        self.start()
        target=[100,100,100,100,100,100,1]
        for k in range(20):
            self.now+=.02
            self.packet('TARGET',target,axis=0)
            before=self.c.q[:]
            self.h.tick(self.now)
            self.assertLessEqual(abs(self.c.q[0]-before[0]),.6+1e-9)
            self.assertEqual(self.c.q[1:],before[1:])
        self.assertAlmostEqual(self.h.velocity[0],30)
        self.assertGreater(self.c.q[0],2)
        self.assertLess(self.c.q[0],12)

    def test_axis_switch_and_hold_never_replay_previous_axis(self):
        self.start()
        for k in range(5):
            self.now+=.02;self.packet('TARGET',[20,20,20,20,20,20,1],0);self.h.tick(self.now)
        j1=self.c.q[0]
        self.now+=.02;self.packet('TARGET',[100,20,20,20,20,20,1],1);self.h.tick(self.now)
        self.assertEqual(self.c.q[0],j1)
        before=self.c.q[:]
        self.now+=.02;self.packet('TARGET',[100,20,20,20,20,20,1],-1);self.h.tick(self.now)
        self.assertEqual(self.c.q,before)

    def test_watchdog_stops_and_does_not_resume_from_late_target(self):
        self.start()
        self.packet('TARGET',[10,0,0,0,0,0,1],0)
        self.now+=.301;self.h.tick(self.now)
        self.assertFalse(self.h.active)
        self.assertTrue(self.c.cancel.is_set())
        before=self.c.q[:]
        self.assertEqual(self.packet('TARGET',[10,0,0,0,0,0,1],0)['command'],'STOP')
        self.h.tick(self.now+.02)
        self.assertEqual(self.c.q,before)

    def test_old_duplicate_nonfinite_and_stale_frames_rejected(self):
        self.start()
        with self.assertRaises(ValueError):
            self.packet('TARGET',[math.nan,0,0,0,0,0,1],0)
        self.assertFalse(self.h.active)
        self.packet('STOP');self.packet('CALIBRATE')
        with self.assertRaises(ValueError): self.packet('TARGET',[0]*7,0,sequence=1)
        with self.assertRaises(ValueError): self.packet('TARGET',[0]*7,0,ageMs=200)
        self.assertFalse(self.h.active)

    def test_initial_open_hand_does_not_release_closed_gripper(self):
        self.c.gripper_preview=True
        self.start(grip=True)
        for k in range(20):
            self.now+=.02;self.packet('TARGET',[0,0,0,0,0,0,1]);self.h.tick(self.now)
        self.assertTrue(self.c.gripper_preview)
        self.assertIsNone(self.h.grip_pending)

    def test_stable_pinch_edges_switch_gripper_without_joint_motion(self):
        self.start(grip=True)
        for grip in (1,0,1):
            for k in range(20):
                self.now+=.02;self.packet('TARGET',[0,0,0,0,0,0,grip]);self.h.tick(self.now)
            self.assertEqual(self.c.gripper_preview,grip==0)
            self.assertEqual(self.c.q,[0]*6)

    def test_stop_and_home_preserve_payload_and_fixed_home(self):
        self.c.gripper_preview=True
        self.start(grip=True)
        home=self.c.home_pose
        self.packet('HOME')
        self.assertTrue(self.h.home_requested)
        self.assertFalse(self.h.active)
        self.assertEqual(self.c.home_pose,home)
        self.assertTrue(self.c.gripper_preview)
        self.assertEqual(self.c.q,[0]*6)

    def test_manual_commands_and_second_camera_are_excluded(self):
        self.start()
        with self.assertRaises(ValueError): self.h.ensure_manual()
        with self.assertRaises(ValueError): self.packet('STOP',session='another-camera')
        self.h.pause(disable=True)
        self.h.ensure_manual()

    def test_real_requires_existing_preparation_and_model_never_uses_robot(self):
        with self.assertRaises(ValueError): self.h.enable(real=True)
        self.make_real()
        with self.assertRaises(ValueError): self.h.enable()
        self.c.armed=False
        with self.assertRaises(ValueError): self.h.enable(real=True,acknowledged=True)

    def test_real_dispatch_bounds_one_joint_and_drops_busy_frames(self):
        self.make_real();self.start(real=True)
        target=[10,20,160,20,40,100,1]
        with patch.object(self.c,'move') as move:
            self.now+=.1;self.packet('TARGET',target,0);self.h.tick(self.now)
            q,revision=move.call_args.args
            self.assertAlmostEqual(q[0]-OBSERVED_POSE[0],.5)
            self.assertEqual(q[1:],OBSERVED_POSE[1:])
            self.assertEqual(revision,self.c.pose_revision)
            self.c.busy=True
            for k in range(30):
                self.now+=.02;self.packet('TARGET',[11,21,161,21,41,101,1],0);self.h.tick(self.now)
            self.assertEqual(move.call_count,1)
            self.c.busy=False
            self.now+=.02;self.packet('TARGET',target,-1);self.h.tick(self.now)
            self.assertEqual(move.call_count,1)

    def test_watchdog_cancels_real_operation_and_never_sends_servo_or_home(self):
        self.make_real();self.start(real=True)
        self.c.busy=True
        with patch.object(self.c,'stop') as stop,patch.object(self.c,'home') as home,patch.object(self.c,'set_servo') as servo:
            self.now+=.31;self.h.tick(self.now)
            stop.assert_called_once();home.assert_not_called();servo.assert_not_called()
            self.assertTrue(self.c.cancel.is_set())
            self.assertFalse(self.h.active)

    def test_gap_cannot_be_hidden_by_a_fresh_packet_before_watchdog_tick(self):
        self.start()
        self.now+=.31
        reply=self.packet('TARGET',[10,0,0,0,0,0,1],0)
        self.assertEqual(reply['command'],'STOP')
        self.assertFalse(self.h.active)

    def test_web_camera_buttons_are_consumed_once_and_disabled_mode_exits(self):
        self.h.enable()
        self.h.camera_input('SPACE')
        link=Link(dry=True)
        link.reply=self.packet('STOP')
        self.assertEqual(link.take_key(),ord(' '))
        self.assertEqual(link.take_key(),-1)
        self.h.camera_input('QUIT')
        link.reply=self.packet('STOP')
        self.assertEqual(link.take_key(),ord('q'))
        self.h.camera_input('SPACE')
        self.h.pause(disable=True)
        self.assertEqual(self.h.input_key,'')
        self.assertEqual(self.packet('STOP')['command'],'EXIT')

    def test_preview_is_bounded_and_kept_only_in_memory(self):
        jpeg=bytes([255,216,0,255,217])
        self.h.camera_preview(base64.b64encode(jpeg).decode())
        self.assertEqual(self.h.preview_jpeg,jpeg)
        self.assertLess(self.h.snapshot()['preview_age'],1)
        for bad in ('not jpeg','A'*170001):
            with self.assertRaises(ValueError): self.h.camera_preview(bad)

    def test_full_tcp_camera_target_moves_only_selected_joint_then_stops(self):
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
        server.daemon_threads=True;server.q=OBSERVED_POSE[:];server.ready=True
        server.start_wait=True;server.exec_delay=.01;server.joint_speed=100;server.commands=[]
        server.manual_step=.5
        threading.Thread(target=server.serve_forever,daemon=True).start()
        self.c.config.update(host='127.0.0.1',port=server.server_address[1],timeout_s=.5,request_terminator='\n')
        client=R3Client(self.c.config);client.connect();self.c.client=client;self.c.mode='monitor'
        self.c._synchronize_pose(client.joints());self.c.heartbeat()
        done=threading.Event()
        def heartbeat():
            while not done.wait(.02): self.c.heartbeat()
        threading.Thread(target=heartbeat,daemon=True).start()
        try:
            self.c.arm(True)
            self.now=time.monotonic();self.start(real=True)
            self.now+=.1
            target=OBSERVED_POSE[:];target[0]+=10
            self.packet('TARGET',target+[1],0);self.h.tick(self.now)
            deadline=time.monotonic()+3
            while self.c.busy and time.monotonic()<deadline: time.sleep(.01)
            self.assertFalse(self.c.busy)
            self.assertAlmostEqual(server.q[0],OBSERVED_POSE[0]+.5,places=4,msg=repr((self.c.events,server.commands,self.h.snapshot())))
            self.assertEqual(server.q[1:],OBSERVED_POSE[1:])
            self.assertEqual(server.commands.count('EXECMov JPCDX'),1)
            self.assertNotIn('SRVON',server.commands)
            self.assertFalse(any(x.startswith('EXECH') for x in server.commands))
            self.packet('STOP')
            self.assertFalse(self.c.armed)
            self.assertEqual(server.commands[-2:],['STOP','CNTLOFF'])
        finally:
            done.set();client.close();self.c.client=None
            server.shutdown();server.server_close()


if __name__=='__main__':
    unittest.main()
