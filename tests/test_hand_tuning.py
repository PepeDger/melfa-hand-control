"""High gain, optional larger hand steps and automatic Z7 synchronization."""
import math
import socketserver
import threading
import time
import unittest
from unittest.mock import patch
from control import Controller, R3Client, load_config, MotionBlocked
from diagnostics import parse_state
from gripper import parse_hands
from hand_control import HandControl
from hand_settings import validate_sensitivity,validate_hand_step
from hand_tracking.direct_core import Mapper,FIELDS
from test_control import OBSERVED_POSE,READY_STATE,ready_status
from serve_fake import Robot


def frame(x=.5,grip=1):
    return dict(x=x,y=.5,size=.15,q=[0,0,0,1],grip=grip,gesture='MOVE')


class TuningTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config());self.c.closed=True
        self.h=HandControl(self.c,start_worker=False,whole_target=False)
        self.now=time.monotonic();self.seq=0

    def tearDown(self):
        self.c.cancel.set()

    def packet(self,command,grip=1):
        self.seq+=1
        return self.h.packet(dict(version=4,session='tuning',sequence=self.seq,action=self.seq,
            command=command,sentMs=int(time.time()*1000),pose=self.c.q[:]+[grip],axis=-1),self.now)

    def ready(self):
        controller=self.c
        class Peer:
            def __init__(self):self.q=OBSERVED_POSE[:];self.closed=False;self.calls=[];self.reads=0;self.moved=False;self.feedback_status_reads=0
            def status(self):
                if self.moved:self.feedback_status_reads+=1
                return parse_state(READY_STATE)
            def joints(self):return self.q[:]
            def close(self):pass
            def hands(self):
                self.reads+=1
                return parse_hands(f'900;{2 if self.closed else 1};1;'+';'.join(['-1']*21))
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd.startswith('EXECJPCDX='):self.target=[float(x) for x in cmd.split('(',1)[1][:-1].split(',')]
                if cmd=='EXECMov JPCDX':self.q=self.target[:];self.moved=True
                if cmd=='EXECHClose 1':self.closed=True
                if cmd=='EXECHOpen 1':self.closed=False
        self.peer=Peer();self.c.client=self.peer;self.c.mode='monitor';self.c.armed=True
        self.c.heartbeat();ready_status(self.c);self.c._synchronize_pose(OBSERVED_POSE)

    def test_gain_and_step_reject_bad_values(self):
        self.assertEqual(validate_sensitivity(50),50)
        for x in (True,0,.49,50.1,'50',math.nan,math.inf):
            with self.assertRaises(ValueError):validate_sensitivity(x)
        for x in (True,.09,5.01,'5',math.inf):
            with self.assertRaises(ValueError):validate_hand_step(x)

    def test_high_gain_increases_travel_without_amplifying_axis_selection_noise(self):
        pose=dict(zip(FIELDS,OBSERVED_POSE));travel=[]
        for gain in (1,20,50):
            mapper=Mapper(gain);mapper.calibrate(frame(),pose,0)
            for k in range(1,35):out,axis=mapper.update(frame(.5+k*.001),k/60)
            self.assertEqual(axis,0);travel.append(out[0]-OBSERVED_POSE[0])
            self.assertEqual(out[1:6],OBSERVED_POSE[1:])
        self.assertGreater(travel[1],travel[0]*3)
        mapper=Mapper(50);mapper.calibrate(frame(),pose,0)
        for k in range(1,80):
            out,axis=mapper.update(frame(.5+(.0003 if k%2 else -.0003)),k/60)
            self.assertEqual(axis,-1);self.assertEqual(out[:6],OBSERVED_POSE)

    def test_live_gain_is_returned_to_camera_and_step_requires_pause(self):
        self.h.enable();self.packet('STOP');self.packet('CALIBRATE')
        self.h.set_options(sensitivity=30)
        self.assertEqual(self.packet('TARGET')['sensitivity'],30)
        with self.assertRaises(ValueError):self.h.set_options(step_deg=2)
        self.packet('STOP');self.h.set_options(step_deg=2)
        self.assertEqual(self.h.snapshot()['real_step_deg'],2)
        self.assertEqual(self.c.hand_step_limit,2)

    def test_hand_fast_path_reuses_own_prepared_controls_but_confirms_three_feedback_samples(self):
        self.ready();self.c.prepared_motion_controls=(self.c.override_percent,self.c.joint_speed)
        target=OBSERVED_POSE[:];target[0]+=.5;delays=[]
        with patch.object(self.c.cancel,'wait',side_effect=lambda dt:delays.append(dt) or False):
            self.assertTrue(self.c._execute_real_step(target,hand_step=True))
        self.assertEqual(self.peer.calls.count('EXECMov JPCDX'),1)
        self.assertNotIn('EXECAccel 1,1',self.peer.calls)
        self.assertNotIn('EXECJOvrd 100',self.peer.calls)
        self.assertNotIn('OVRD=1',self.peer.calls)
        self.assertEqual(sum(abs(dt-.1)<1e-9 for dt in delays),1)
        self.assertEqual(delays[-3:],[.1,0,0])
        self.assertEqual(self.peer.feedback_status_reads,3)
        for a,b in zip(self.c.q,target):self.assertAlmostEqual(a,b,places=5)

    def test_fast_hand_path_rechecks_pose_and_respects_step_limit(self):
        self.ready();self.c.prepared_motion_controls=(1,100)
        target=OBSERVED_POSE[:];target[0]+=2
        with self.assertRaises(MotionBlocked):self.c._execute_real_step(target,hand_step=True)
        self.assertEqual(self.peer.calls,[])
        self.c.hand_step_limit=2
        self.peer.q[0]+=.02
        with self.assertRaises(MotionBlocked):self.c._execute_real_step(target,hand_step=True)
        self.assertNotIn('EXECMov JPCDX',self.peer.calls)

    def test_pending_move_rebases_future_goal_without_falsifying_encoder_pose(self):
        self.ready();self.h.real=True
        self.c.busy=True;self.c.operation='move';self.c.target=OBSERVED_POSE[:];self.c.target[0]+=2
        reply=self.h._reply(dict(session='test',sequence=1,action=1),'TARGET','Direct coalesced / latest target')
        self.assertEqual(list(reply['pose'].values()),OBSERVED_POSE)
        mapper=Mapper(20);mapper.calibrate(frame(),reply['pose'],0);mapper.active=0
        mapper.reconcile(reply)
        self.assertEqual(mapper.actual,OBSERVED_POSE)
        self.assertEqual(mapper.goal,self.c.target)
        out,axis=mapper.update(frame(.51),.02)
        self.assertEqual(axis,0);self.assertGreater(out[0],self.c.target[0])
        self.assertEqual(out[1:6],OBSERVED_POSE[1:])

    def test_invalid_pending_reference_cannot_change_goal_to_nonfinite_or_outside_factory(self):
        mapper=Mapper();pose=dict(zip(FIELDS,OBSERVED_POSE));mapper.calibrate(frame(),pose,0)
        for bad in (math.nan,1e5,True):
            ref=pose.copy();ref[FIELDS[0]]=bad
            mapper.reconcile(dict(pose=pose,hand_reference=ref,ikStatus='Direct coalesced'))
            self.assertEqual(mapper.goal,OBSERVED_POSE)

    def test_short_reverse_noise_is_held_but_intentional_reverse_is_accepted_after_30ms(self):
        mapper=Mapper(20);mapper.calibrate(frame(),dict(zip(FIELDS,OBSERVED_POSE)),0)
        mapper.active=0;mapper.drive_direction=1
        for t,x in ((.02,.49),(.04,.48)):
            out,axis=mapper.update(frame(x),t)
            self.assertEqual(axis,0);self.assertEqual(out[0],OBSERVED_POSE[0])
        out,axis=mapper.update(frame(.47),.06)
        self.assertLess(out[0],OBSERVED_POSE[0]);self.assertEqual(out[1:6],OBSERVED_POSE[1:])

    def test_moving_feedback_still_waits_100ms_until_target_and_stop(self):
        self.ready();self.c.prepared_motion_controls=(1,100)
        peer=self.peer;original=peer.status
        def status():
            if peer.moved and peer.feedback_status_reads==0:
                peer.feedback_status_reads+=1
                return parse_state(READY_STATE.replace('A1120','E1000'))
            return original()
        peer.status=status;delays=[];target=OBSERVED_POSE[:];target[0]+=.5
        with patch.object(self.c.cancel,'wait',side_effect=lambda dt:delays.append(dt) or False):
            self.assertTrue(self.c._execute_real_step(target,hand_step=True))
        self.assertEqual(delays[-4:],[.1,.1,0,0])
        self.assertEqual(peer.feedback_status_reads,4)

    def test_stop_between_final_confirmations_cancels_without_another_motion(self):
        self.ready();self.c.prepared_motion_controls=(1,100)
        target=OBSERVED_POSE[:];target[0]+=.5
        def wait(dt):
            if dt==0:self.c.cancel.set();return True
            return False
        with patch.object(self.c.cancel,'wait',side_effect=wait):
            self.assertFalse(self.c._execute_real_step(target,hand_step=True))
        self.assertEqual(self.peer.calls.count('EXECMov JPCDX'),1)
        self.assertEqual(self.peer.feedback_status_reads,1)

    def test_grip_toggle_automatically_reads_status_without_actuation(self):
        self.ready();self.h.enable(real=True,gripper=True,acknowledged=True)
        self.assertTrue(self.h.grip_sync_requested)
        self.h.tick(self.now)
        deadline=time.monotonic()+1
        while self.c.busy and time.monotonic()<deadline:time.sleep(.005)
        self.h.tick(self.now+.02)
        self.assertTrue(self.h.grip_ready)
        self.assertGreater(self.peer.reads,0)
        self.assertEqual(self.peer.calls,[])
        self.h.set_options(gripper=False)
        self.assertFalse(self.h.grip_enabled)
        self.assertIsNone(self.h.grip_pending)

    def test_first_closed_pinch_requires_open_baseline_and_never_drops_payload(self):
        self.c.gripper_preview=True
        self.h.enable(gripper=True);self.packet('STOP');self.packet('CALIBRATE')
        for grip in (0,1):
            for k in range(20):self.now+=.02;self.packet('TARGET',grip);self.h.tick(self.now)
            self.assertTrue(self.c.gripper_preview)
        for grip in (0,1):
            for k in range(20):self.now+=.02;self.packet('TARGET',grip);self.h.tick(self.now)
            self.assertEqual(self.c.gripper_preview,grip==0)

    def test_active_z7_toggle_resets_baseline_without_restarting_camera_or_moving_axes(self):
        self.h.enable();self.packet('STOP');self.packet('CALIBRATE')
        self.h.set_options(gripper=True)
        for grip in (1,0):
            for k in range(20):self.now+=.02;self.packet('TARGET',grip);self.h.tick(self.now)
        self.assertTrue(self.c.gripper_preview)
        self.assertEqual(self.c.q,[0]*6)
        self.assertTrue(self.h.active)
        self.h.set_options(gripper=False)
        for k in range(20):self.now+=.02;self.packet('TARGET',1);self.h.tick(self.now)
        self.assertTrue(self.c.gripper_preview)

    def test_full_tcp_auto_read_and_pinch_close_open_without_separate_read_button(self):
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
        server.daemon_threads=True;server.q=OBSERVED_POSE[:];server.ready=True
        server.start_wait=True;server.exec_delay=.02;server.joint_speed=100;server.commands=[]
        threading.Thread(target=server.serve_forever,daemon=True).start()
        self.c.config.update(host='127.0.0.1',port=server.server_address[1],timeout_s=.5,request_terminator='\n')
        client=R3Client(self.c.config);client.connect();self.c.client=client;self.c.mode='monitor'
        self.c._synchronize_pose(client.joints());self.c.heartbeat()
        done=threading.Event()
        def feedback():
            while not done.wait(.02):
                self.c.heartbeat()
                # The fixture has fixed joints: simulate idle monitor freshness.
                with self.c.lock:
                    if not self.c.busy:self.c.feedback_at=time.monotonic()
        threading.Thread(target=feedback,daemon=True).start()
        try:
            self.c.arm(True)
            self.h.enable(real=True,gripper=True,acknowledged=True)
            self.now=time.monotonic();self.packet('STOP');self.packet('CALIBRATE')
            for grip in (1,0,1):
                deadline=time.monotonic()+.8
                while time.monotonic()<deadline:
                    self.now=time.monotonic();self.packet('TARGET',grip);self.h.tick(self.now);time.sleep(.02)
                if grip!=1 or 'EXECHClose 1' in server.commands:
                    self.assertEqual(getattr(server,'hand_closed',False),grip==0)
            self.assertEqual(server.commands.count('EXECHClose 1'),1)
            self.assertEqual(server.commands.count('EXECHOpen 1'),1)
            self.assertIn('HNDSTS',server.commands)
            self.assertNotIn('EXECMov JPCDX',server.commands)
            self.assertNotIn('SRVON',server.commands)
            self.assertEqual(server.q,OBSERVED_POSE)
            self.assertTrue(self.h.grip_ready)
        finally:
            self.c.cancel.set();done.set()
            deadline=time.monotonic()+1
            while self.c.busy and time.monotonic()<deadline:time.sleep(.01)
            client.close();self.c.client=None
            server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
