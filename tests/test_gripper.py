"""Binary Z7/Hand 1 controls, status framing and output feedback; no hardware."""
import socketserver
import threading
import time
import unittest
from unittest.mock import patch
from control import Controller, R3Client, load_config, validate_pose
from gripper import parse_hands
from test_control import FakeRobot, OBSERVED_POSE, READY_STATE, ready_status
from diagnostics import parse_state
from serve_fake import Robot

OPEN='900;1;1;'+';'.join(['-1']*21)
CLOSED='900;2;1;'+';'.join(['-1']*21)

class GripperTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(load_config());self.c.closed=True
    def tearDown(self):
        self.c.cancel.set()
        if self.c.client: self.c.client.close()

    def peer(self,unused=False,ignore=False):
        class Peer:
            def __init__(self): self.closed=False;self.calls=[]
            def hands(self):
                return parse_hands(';'.join(['-1']*24) if unused else CLOSED if self.closed else OPEN)
            def status(self): return parse_state(READY_STATE)
            def command(self,cmd):
                self.calls.append(cmd)
                if not ignore and cmd=='EXECHClose 1': self.closed=True
                if not ignore and cmd=='EXECHOpen 1': self.closed=False
            def close(self): pass
        self.c.client=Peer();self.c.mode='monitor';self.c.armed=True
        self.c.operator_session=True;self.c.heartbeat();ready_status(self.c)
        self.c._synchronize_pose(OBSERVED_POSE)
        return self.c.client

    def test_documented_status_has_eight_channels_not_joint_positions(self):
        records=parse_hands(CLOSED)
        self.assertEqual(len(records),8)
        self.assertEqual(records[0],dict(hand=1,output=900,output_state=2,solenoid_type=1,closed=True))
        self.assertIsNone(records[-1]['closed'])
        for bad in (OPEN+';junk',OPEN.rsplit(';',1)[0],OPEN.replace('900;1;1','900;3;1'),OPEN.replace('900;1;1','-1;1;1')):
            with self.assertRaises(ValueError): parse_hands(bad)

    def test_fragmented_hndsts_does_not_end_at_a_tcp_pause(self):
        robot=FakeRobot([[b'QoK'+OPEN[:17].encode(),(.2,OPEN[17:].encode())],[b'QoK']])
        client=robot.client();client.config.update(reply_terminator='',timeout_s=.8)
        self.assertFalse(client.hands()[0]['closed'])
        self.assertEqual(client.command('CNTLOFF'),'');client.close()

    def test_invalid_hand_configuration_is_rejected(self):
        self.assertEqual(self.c.gripper_hand,1)
        for hand in (True,0,9,'1'):
            config=load_config();config['gripper_hand']=hand
            with self.assertRaises(ValueError): Controller(config)

    def test_malformed_status_closes_transport_without_other_commands(self):
        robot=FakeRobot([[b'QoK'+OPEN.encode()+b';bad\r\n']])
        client=robot.client()
        with self.assertRaises(ValueError): client.hands()
        self.assertIsNone(client.sock)
        self.assertEqual(robot.commands,[b'1;1;HNDSTS'])

    def test_simulation_does_not_add_joint_or_touch_real_outputs(self):
        q=self.c.q[:]
        self.c.set_gripper(True)
        self.assertTrue(self.c.snapshot()['gripper']['preview_closed'])
        self.assertEqual(self.c.q,q)
        self.assertEqual(len(self.c.q),6)
        with self.assertRaises(ValueError): validate_pose(q+[1])

    def test_explicit_commands_use_hand_one_and_feedback_not_mov(self):
        peer=self.peer()
        with patch.object(self.c.cancel,'wait',side_effect=lambda _:self.c.cancel.is_set()):
            self.c.set_gripper(True,self.c.pose_revision)
            self.assertTrue(self.c.snapshot()['gripper']['closed'])
            self.c.set_gripper(True,self.c.pose_revision)  # no duplicate valve command
            self.c.set_gripper(False,self.c.pose_revision)
        self.assertEqual(peer.calls,['EXECHClose 1','EXECHOpen 1'])
        self.assertEqual(self.c.q,OBSERVED_POSE)
        self.assertFalse(any('Mov' in cmd for cmd in peer.calls))

    def test_unassigned_hand_and_unprepared_or_busy_session_do_not_actuate(self):
        peer=self.peer(unused=True)
        with self.assertRaises(ValueError): self.c.set_gripper(True,self.c.pose_revision)
        self.assertFalse(any(cmd.startswith('EXECH') for cmd in peer.calls))
        peer=self.peer();self.c.armed=False
        with self.assertRaises(ValueError): self.c.set_gripper(True,self.c.pose_revision)
        self.c.armed=True;self.c.busy=True
        with self.assertRaises(ValueError): self.c.set_gripper(True,self.c.pose_revision)
        self.c.busy=False
        with self.assertRaises(ValueError): self.c.set_gripper(50,self.c.pose_revision)
        self.assertEqual(peer.calls,[])

    def test_ack_without_output_change_cancels_without_retry(self):
        peer=self.peer(ignore=True)
        def wait(delay):
            return delay==.05  # cancel after first output mismatch
        with patch.object(self.c.cancel,'wait',side_effect=wait):
            with self.assertRaises(ValueError): self.c.set_gripper(True,self.c.pose_revision)
        self.assertEqual(peer.calls.count('EXECHClose 1'),1)
        self.assertNotIn('EXECHOpen 1',peer.calls)
        self.assertFalse(self.c.armed)

    def test_stop_never_releases_a_closed_gripper(self):
        peer=self.peer();peer.closed=True
        self.c._read_gripper_status();self.c.stop()
        self.assertTrue(peer.closed)
        self.assertEqual(peer.calls,['STOP','CNTLOFF'])

    def test_full_tcp_open_close_are_confirmed_without_arm_movement(self):
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
        server.daemon_threads=True;server.q=OBSERVED_POSE[:];server.ready=True
        server.start_wait=True;server.exec_delay=.02;server.joint_speed=100;server.commands=[]
        threading.Thread(target=server.serve_forever,daemon=True).start()
        self.c.config.update(host='127.0.0.1',port=server.server_address[1],timeout_s=1,request_terminator='\n')
        client=R3Client(self.c.config);client.connect();self.c.client=client;self.c.mode='monitor'
        self.c._synchronize_pose(client.joints())
        self.c.heartbeat()
        alive=threading.Event()
        def heartbeat():
            while not alive.wait(.02): self.c.heartbeat()
        beat=threading.Thread(target=heartbeat,daemon=True);beat.start()
        try:
            self.c.read_gripper();self.c.arm(True)
            for closed in (True,False):
                self.c.set_gripper(closed,self.c.pose_revision)
                self.assertEqual(self.c.snapshot()['gripper']['closed'],closed)
                self.assertEqual(client.joints(),OBSERVED_POSE)
            self.assertEqual(server.commands.count('EXECHClose 1'),1)
            self.assertEqual(server.commands.count('EXECHOpen 1'),1)
            self.assertNotIn('EXECMov JPCDX',server.commands)
        finally:
            alive.set();beat.join(1);client.close();self.c.client=None
            server.shutdown();server.server_close()

if __name__=='__main__': unittest.main()
