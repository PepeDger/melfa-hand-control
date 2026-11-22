"""Regression coverage for the CR2D STOP+WAIT frames seen after EXEC setup.

The complete happy path uses a loopback TCP controller which rejects queued
EXEC commands while busy, echoes QoK1 and retains WAIT after each instruction.
"""
import socketserver
import threading
import time
import unittest

from control import Controller, MotionBlocked, R3Client, load_config
from diagnostics import movement_blockers, parse_state
from serve_fake import Robot, POSE
from test_control import READY_STATE, ready_status

WAIT_STATE = READY_STATE.replace('A1120','A1060')


class WaitFlowTests(unittest.TestCase):
    def setUp(self):
        config=load_config(); config['joint_speed_percent']=10
        self.c=Controller(config); self.c.closed=True
        self.c.heartbeat()

    def tearDown(self):
        self.c.cancel.set()
        deadline=time.monotonic()+3
        while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
        if self.c.client: self.c.client.close()

    def test_stopped_wait_requires_operator_session_and_does_not_allow_running(self):
        state=parse_state(WAIT_STATE)
        self.assertTrue(state['waiting']); self.assertTrue(state['stopped'])
        self.assertTrue(movement_blockers(state))
        self.assertEqual(movement_blockers(state,operator_session=True),[])
        for key,value in dict(running=True,program_running=True,editing=True,
                              stopped=False,servo_on=False,error=123,emergency_stop=True,
                              operation_enabled=False,automatic=False).items():
            with self.subTest(key=key):
                changed=dict(state,**{key:value})
                self.assertTrue(movement_blockers(changed,operator_session=True))

    def test_full_tcp_prepare_move_and_release_with_latched_wait(self):
        robot=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
        robot.daemon_threads=True; robot.q=POSE[:]; robot.ready=True
        robot.start_wait=True; robot.exec_delay=.05; robot.commands=[]
        robot.manual_step=20.; self.c.manual_step=20.; self.c.joint_speed=100; robot.joint_speed=100
        threading.Thread(target=robot.serve_forever,daemon=True).start()
        self.c.config.update(host='127.0.0.1',port=robot.server_address[1],timeout_s=.5,request_terminator='\n')
        client=R3Client(self.c.config); client.connect()
        self.c.client=client; self.c.mode='monitor'
        self.c._synchronize_pose(client.joints())
        alive=threading.Event()
        def heartbeat():
            while not alive.wait(.02): self.c.heartbeat()
        beat=threading.Thread(target=heartbeat,daemon=True); beat.start()
        try:
            self.c.arm(True)
            self.assertTrue(self.c.armed)
            self.assertTrue(self.c.controller_status['waiting'])
            self.assertEqual(self.c.snapshot()['movement_blockers'],[])
            for percent in (1,25,100):
                self.c.set_speed(percent,self.c.pose_revision)
                self.assertEqual(self.c.controller_status['override'],percent)
                self.assertEqual(self.c.override_percent,percent)
            self.assertNotIn('EXECMov JPCDX',robot.commands)
            goal=POSE[:]; goal[0]+=20.; goal[1]+=5.; goal[4]-=5.
            self.c.move(goal,self.c.pose_revision)
            deadline=time.monotonic()+3
            while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
            self.assertFalse(self.c.busy)
            self.assertTrue(self.c.armed,self.c.message)
            self.assertEqual(self.c.q,goal)
            self.assertIn('завершена',self.c.message)
            self.assertEqual(robot.commands.count('EXECMov JPCDX'),1)
            self.assertIn('EXECJOvrd 100',robot.commands)
            self.assertNotIn('EXECJOvrd 10',robot.commands)
            self.assertEqual(self.c.speed_commands[-1],'EXECAccel 1,1')
            self.assertFalse(any(c in ('START','RUN','SLOTINIT','RSTALRM','SRVON') for c in robot.commands))
            # Same real protocol on loopback: HOME returns the +20° excursion
            # in one coordinated move, without enabling commissioning flags.
            self.c.home(self.c.pose_revision)
            deadline=time.monotonic()+12
            while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
            self.assertFalse(self.c.busy,self.c.message)
            self.assertTrue(self.c.armed,self.c.message)
            self.assertEqual(self.c.q,POSE)
            self.assertIn('сохранённая поза достигнута',self.c.message)
            self.assertEqual(robot.commands.count('EXECMov JPCDX'),2)
            self.assertFalse(self.c.config['motion_commissioned'])
            self.c.stop()
            self.assertFalse(self.c.armed)
            self.assertFalse(self.c.operator_session)
            self.assertEqual(robot.commands[-2:],['STOP','CNTLOFF'])
        finally:
            alive.set(); beat.join(1)
            client.close(); self.c.client=None
            robot.shutdown(); robot.server_close()

    def test_other_program_change_aborts_setup_and_releases_control(self):
        class Peer:
            def __init__(self): self.calls=[]; self.changed=False
            def status(self):
                result=parse_state(WAIT_STATE)
                if self.changed: result['program']='OTHER.MB5'
                return result
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd=='EXECJOvrd 10': self.changed=True
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'
        self.c._synchronize_pose(POSE)
        with self.assertRaisesRegex(MotionBlocked,'программа|строка'):
            self.c.arm(True)
        self.assertEqual(peer.calls[-1],'CNTLOFF')
        self.assertNotIn('EXECAccel 1,1',peer.calls)
        self.assertFalse(self.c.armed); self.assertFalse(self.c.operator_session)
        self.assertEqual(self.c.mode,'monitor')

    def test_cancel_after_final_pose_read_prevents_mov(self):
        controller=self.c
        class Peer:
            def __init__(self): self.calls=[]; self.reads=0
            def status(self): return parse_state(WAIT_STATE)
            def joints(self):
                self.reads+=1
                if self.reads==2: controller.cancel.set()
                return POSE[:]
            def command(self,cmd): self.calls.append(cmd)
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'; self.c.armed=True
        self.c.operator_session=True
        self.c._synchronize_pose(POSE)
        goal=POSE[:]; goal[0]+=.1
        with self.assertRaisesRegex(MotionBlocked,'отменена'):
            self.c._execute_real_step(goal)
        self.assertNotIn('EXECMov JPCDX',peer.calls)

    def test_rechecking_connection_requires_stopping_active_control(self):
        self.c.armed=True
        with self.assertRaisesRegex(ValueError,'STOP'): self.c.check_connection()

    def test_heartbeat_loss_during_setup_releases_rights_without_motion(self):
        controller=self.c
        class Peer:
            def __init__(self): self.calls=[]
            def status(self): return parse_state(WAIT_STATE)
            def command(self,cmd):
                self.calls.append(cmd)
                if cmd=='EXECJOvrd 10': controller.heartbeat_at-=2
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'
        self.c._synchronize_pose(POSE)
        with self.assertRaisesRegex(MotionBlocked,'окном оператора'):
            self.c.arm(True)
        self.assertEqual(peer.calls[-1],'CNTLOFF')
        self.assertNotIn('EXECAccel 1,1',peer.calls)
        self.assertNotIn('EXECMov JPCDX',peer.calls)
        self.assertFalse(self.c.armed)
        self.assertFalse(self.c.operator_session)

    def test_guard_failure_preserves_read_only_channel_and_resynchronizes(self):
        class Peer:
            def __init__(self): self.calls=[]
            def command(self,cmd): self.calls.append(cmd)
            def joints(self): return POSE[:]
            def close(self): pass
        peer=Peer(); self.c.client=peer; self.c.mode='monitor'; self.c.armed=True
        self.c.busy=True; self.c.operation='move'
        def blocked(target): raise MotionBlocked('проверяемый запрет')
        self.c._execute_real_step=blocked
        self.c._move(POSE[:])
        self.assertEqual(peer.calls,['STOP','CNTLOFF'])
        self.assertEqual(self.c.mode,'monitor')
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.q,self.c.target)
        self.assertIn('доступно чтение',self.c.message)


if __name__=='__main__': unittest.main()
