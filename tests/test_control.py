import copy
import math
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
import tempfile
import atexit
from unittest.mock import patch

# Unit tests must never inherit or overwrite the operator's saved joint limits.
_limits_temp=tempfile.TemporaryDirectory()
_limits_patch=patch('control.DEFAULT_LIMITS_PATH',Path(_limits_temp.name)/'motion_limits.json')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control import Controller, ProtocolError, R3Client, load_config, parse_joints, validate_pose
_limits_patch.start()
atexit.register(_limits_temp.cleanup)
atexit.register(_limits_patch.stop)
from diagnostics import parse_dstate, parse_state, movement_blockers

OBSERVED_PAYLOAD = ('J1;1.53;J2;-8.26;J3;157.49;J4;-0.01;J5;30.79;J6;91.52;'
                    ';****,****;100;0.00;00000000')
OBSERVED_POSE = [1.53, -8.26, 157.49, -.01, 30.79, 91.52]
OBSERVED_STATE = 'MAIN.MB5;1;100;0;01120;1;1;;;;;;;;;REP;START;1;0'
READY_STATE = OBSERVED_STATE.replace(';100;', ';1;').replace('01120', 'A1120')


def ready_status(controller):
    controller.controller_status = parse_state(READY_STATE)
    controller.status_at = time.monotonic()


def frame(q=(0, 0, 90, 0, 0, 0)):
    return ''.join(f'J{i+1};{x};' for i,x in enumerate(q))


class FakeRobot:
    """Actual loopback TCP peer, with fragmented responses."""
    def __init__(self, replies):
        self.replies = replies
        self.commands = []
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', 0))
        self.port = self.server.getsockname()[1]
        self.server.listen(1)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            conn, _ = self.server.accept()
            with conn:
                conn.settimeout(2)
                for fragments in self.replies:
                    self.commands.append(conn.recv(4096))
                    for fragment in fragments:
                        if isinstance(fragment, tuple):
                            delay, fragment = fragment
                            time.sleep(delay)
                        conn.sendall(fragment)
                        time.sleep(.01)
        except (OSError, TimeoutError):
            pass
        finally:
            self.server.close()

    def client(self):
        cfg = load_config()
        cfg.update(host='127.0.0.1', port=self.port, reply_terminator='\r\n', timeout_s=.3,
                   jposf_reply_format='bare')
        c = R3Client(cfg)
        c.connect()
        return c


class ValidationTests(unittest.TestCase):
    def test_exact_observed_cr2d_reply(self):
        self.assertEqual(parse_joints(OBSERVED_PAYLOAD, 'cr2d_extended'), OBSERVED_POSE)
        self.assertEqual(parse_joints(OBSERVED_PAYLOAD), OBSERVED_POSE)

    def test_service_numbers_never_become_joint_values(self):
        reply=OBSERVED_PAYLOAD.replace(';100;0.00;00000000', ';12.5;-999.25;A0B1C2D3')
        self.assertEqual(parse_joints(reply, 'cr2d_extended'), OBSERVED_POSE)

    def test_extended_profile_requires_complete_valid_trailer(self):
        bad = [OBSERVED_PAYLOAD[:-1], OBSERVED_PAYLOAD+'0', OBSERVED_PAYLOAD+';QeR1234',
               OBSERVED_PAYLOAD.replace('****,****', 'junk'),
               OBSERVED_PAYLOAD.replace(';100;', ';1e999;'),
               OBSERVED_PAYLOAD.replace('J6;91.52;', ''),
               OBSERVED_PAYLOAD.replace('J6;91.52;', 'J5;91.52;'), frame()]
        for reply in bad:
            with self.subTest(reply=reply), self.assertRaises(ValueError):
                parse_joints(reply, 'cr2d_extended')

    def test_valid_six_and_eight_axes(self):
        self.assertEqual(parse_joints(frame()), [0,0,90,0,0,0])
        self.assertEqual(parse_joints(frame()+'J7;0;J8;0;'), [0,0,90,0,0,0])

    def test_missing_duplicate_nonfinite_and_tail_rejected(self):
        for s in (frame()[:-1], frame().replace('J6','J5'), frame().replace('90','nan'),
                  frame()+'QeR1234', frame()+'J7;0;', '1;2;3;4;5;6;'):
            # Seven labelled axes is permissible if J1..J6 are complete.
            if s.endswith('J7;0;'): continue
            with self.subTest(s=s), self.assertRaises(ValueError): parse_joints(s)

    def test_limits_and_malformed_input(self):
        for v in ([0]*5, [0,0,172,0,0,0], [math.nan]*6, [math.inf]*6,
                  [True]*6, ['0']*6, None):
            with self.subTest(v=v), self.assertRaises(ValueError): validate_pose(v)

    def test_boundary_angles(self):
        self.assertEqual(validate_pose([-170,-90,-20,-160,-120,-360])[0], -170)
        self.assertEqual(validate_pose([170,135,171,160,120,360])[-1], 360)


class TransportTests(unittest.TestCase):
    def test_observed_override_echo_keeps_connection_and_reads_next_reply(self):
        robot=FakeRobot([[b'QoK1\r\n'],[b'QoK'+frame().encode()+b'\r\n']])
        c=robot.client()
        self.assertEqual(c.command('OVRD=1'),'1')
        self.assertEqual(c.joints(),[0,0,90,0,0,0])
        self.assertEqual(robot.commands,[b'1;1;OVRD=1',b'1;1;JPOSF'])
        c.close()

    def test_override_echo_fragmented_after_qok_is_not_an_empty_ack(self):
        robot=FakeRobot([[b'QoK',(.2,b'1')],[b'QoK']])
        c=robot.client(); c.config.update(reply_terminator='',timeout_s=.8)
        self.assertEqual(c.command('OVRD=1'),'1')
        self.assertEqual(c.command('CNTLOFF'),'')
        c.close()

    def test_wrong_override_echo_is_rejected_without_retry(self):
        for reply in (b'QoK100\r\n',b'QoK0\r\n',b'QoK\r\n',b'QoK1junk\r\n'):
            robot=FakeRobot([[reply]])
            c=robot.client()
            with self.assertRaises(ValueError): c.command('OVRD=1')
            self.assertIsNone(c.sock)
            self.assertEqual(robot.commands,[b'1;1;OVRD=1'])

    def test_echo_exception_does_not_apply_to_other_commands(self):
        robot=FakeRobot([[b'QoK1\r\n']]); c=robot.client()
        with self.assertRaises(ValueError): c.command('CNTLON')
        self.assertIsNone(c.sock)

    def test_full_observed_frame_without_line_ending(self):
        robot=FakeRobot([[b'QoK'+OBSERVED_PAYLOAD.encode()]])
        c=robot.client()
        c.config['reply_terminator']=''
        c.jposf_format='cr2d_extended'
        self.assertEqual(c.joints(), OBSERVED_POSE)
        c.close()
        self.assertEqual(robot.commands,[b'1;1;JPOSF'])

    def test_delayed_metadata_is_not_mistaken_for_end_of_frame(self):
        head,tail=OBSERVED_PAYLOAD.split(';;',1)
        robot=FakeRobot([[b'QoK'+head.encode()+b';',(.20,b';'+tail[:-4].encode()),tail[-4:].encode()]])
        c=robot.client()
        c.config.update(reply_terminator='',timeout_s=.8)
        c.jposf_format='cr2d_extended'
        self.assertEqual(c.joints(),OBSERVED_POSE)
        c.close()

    def test_fragmented_joint_reply(self):
        robot = FakeRobot([[b'Q', b'oKJ1;0;J2;', b'0;J3;90;J4;0;', b'J5;0;J6;0;\r', b'\n']])
        c = robot.client()
        self.assertEqual(c.joints(), [0,0,90,0,0,0])
        c.close()
        self.assertEqual(robot.commands, [b'1;1;JPOSF'])

    def test_error_closes_socket_no_retry(self):
        robot = FakeRobot([[b'QeR', b'6010\r\n']])
        c = robot.client()
        with self.assertRaises(ValueError): c.command('CNTLON')
        self.assertIsNone(c.sock)
        self.assertEqual(len(robot.commands), 1)

    def test_partial_reply_times_out_and_disconnects(self):
        robot = FakeRobot([[b'QoKJ1;0;']])
        c = robot.client()
        with self.assertRaises((ConnectionError, TimeoutError)): c.joints()
        self.assertIsNone(c.sock)

    def test_coalesced_extra_reply_is_rejected(self):
        robot = FakeRobot([[b'QoK\r\nQoK\r\n']])
        c = robot.client()
        with self.assertRaises(ValueError): c.command('STOP')
        self.assertIsNone(c.sock)

    def test_idle_gap_legacy_reply(self):
        robot = FakeRobot([[b'QoK'+frame().encode()], [b'QoK']])
        c = robot.client()
        c.config['reply_terminator'] = ''
        self.assertEqual(c.joints(), [0,0,90,0,0,0])
        c.close()


class SafetyStateTests(unittest.TestCase):
    def setUp(self):
        config=copy.deepcopy(load_config()); config.update(manual_step_deg=2.,joint_speed_percent=10)
        self.c = Controller(config)

    def tearDown(self):
        self.c.closed = True
        self.c.cancel.set()
        if self.c.client: self.c.client.close()

    def test_default_no_network_or_motion_permission(self):
        self.assertFalse(self.c.armed)
        self.assertIsNone(self.c.client)
        self.assertEqual(self.c.mode, 'simulation')

    def test_read_only_connect_sends_only_jposf(self):
        robot = FakeRobot([[b'QoK'+frame().encode()+b'\r\n'], [b'QoK'+frame().encode()+b'\r\n']])
        self.c.config.update(host='127.0.0.1', port=robot.port, reply_terminator='\r\n',
                             jposf_reply_format='bare')
        self.c.connect()
        deadline=time.monotonic()+2
        while self.c.busy and time.monotonic()<deadline: time.sleep(.01)
        self.assertEqual(self.c.mode, 'monitor')
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.target, self.c.q)
        self.assertTrue(all(cmd == b'1;1;JPOSF' for cmd in robot.commands))

    def test_commissioning_blocks_arm_before_any_command(self):
        self.c.config['physical_control_enabled']=False
        self.c.mode='monitor'
        self.c.client=type('Peer',(),{'close':lambda s:None})()
        self.c.feedback_at=time.monotonic()
        with self.assertRaisesRegex(ValueError,'отключено'): self.c.arm(True)

    def test_real_step_rejects_large_out_of_factory_and_stale(self):
        self.c.q=[0,0,90,0,0,0]
        self.c.pose_synced=True
        self.c.pose_revision=7
        self.c.mode='monitor'
        self.c.armed=True
        self.c.client=type('Peer',(),{'close':lambda s:None})()
        self.c.feedback_at=self.c.heartbeat_at=time.monotonic()
        for target in ([3,0,90,0,0,0],[0,136,90,0,0,0]):
            with self.assertRaises(ValueError): self.c.move(target,7)
        self.c.feedback_at-=5
        with self.assertRaisesRegex(ValueError,'свежего'): self.c.move([1,0,90,0,0,0],7)

    def test_busy_rejects_queued_moves(self):
        self.c.busy=True
        with self.assertRaises(ValueError): self.c.move([1,0,90,0,0,0])

    def test_simulation_moves_smoothly_and_stops(self):
        self.c.move([30,0,90,0,0,0])
        time.sleep(.12)
        self.assertGreater(self.c.q[0],0)
        self.assertLess(self.c.q[0],1)
        self.c.stop()
        time.sleep(.06)
        q=self.c.q[:]
        time.sleep(.06)
        self.assertEqual(q,self.c.q)
        self.assertFalse(self.c.busy)

    def test_work_limits_may_only_be_tightened(self):
        cfg=copy.deepcopy(load_config())
        cfg['joint_limits'][0]=[-180,180]
        with self.assertRaises(ValueError): Controller(cfg)

    def test_permission_is_not_a_truthy_string(self):
        cfg=copy.deepcopy(load_config())
        cfg['motion_commissioned']='false'
        with self.assertRaises(ValueError): Controller(cfg)

    def test_qualified_command_sequence_never_enables_servo(self):
        self.c.closed=True
        class Peer:
            def __init__(self): self.commands=[]
            def command(self,cmd): self.commands.append(cmd); return ''
            def joints(self): return [0]*6
            def status(self): return parse_state(READY_STATE)
            def close(self): pass
        peer=Peer()
        self.c.client=peer
        self.c.config.update(motion_commissioned=True,controller_model='test-only',controller_firmware='fake')
        self.c.mode='monitor'
        self.c.feedback_at=time.monotonic()
        self.c.heartbeat_at=time.monotonic()
        self.c.arm(True)
        self.assertTrue(self.c.armed)
        self.assertEqual(peer.commands,['CNTLON','OVRD=1','EXECJOvrd 10','EXECAccel 1,1'])
        self.c.stop()
        self.assertFalse(self.c.armed)
        self.assertEqual(peer.commands[-2:],['STOP','CNTLOFF'])
        self.assertFalse(any('SRVON' in c or 'Mov' in c for c in peer.commands))


class ConnectionCheckTests(unittest.TestCase):
    class Peer:
        def __init__(self, fail=False): self.fail=fail; self.calls=[]
        def command(self, cmd): self.calls.append(cmd)
        def status(self): return parse_state(READY_STATE)
        def connect(self): self.calls.append('connect')
        def joints(self):
            self.calls.append('JPOSF')
            if self.fail: raise ConnectionError('test failure')
            return [5, 0, 90, 0, 0, 0]
        def close(self): self.calls.append('close')

    def setUp(self):
        self.c=Controller(copy.deepcopy(load_config()))
        self.c.closed=True

    def tearDown(self):
        if self.c.client: self.c.client.close()

    def finish(self):
        deadline=time.monotonic()+2
        while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
        self.assertFalse(self.c.busy)

    def test_connect_check_and_sync_are_read_only(self):
        self.c.q=[120,-50,150,-90,80,-250]
        self.c.target=[-100,70,100,30,90,200]
        peer=self.Peer()
        with patch('control.R3Client',return_value=peer):
            self.c.check_connection()
            self.finish()
        state=self.c.snapshot()
        self.assertEqual(peer.calls,['connect','JPOSF'])
        self.assertEqual(state['connection_check']['status'],'ok')
        self.assertEqual(state['joints'],state['target'])
        self.assertEqual(state['pose_revision'],1)
        self.assertTrue(state['pose_synced'])
        self.assertFalse(state['armed'])

    def test_existing_connection_rechecked_without_new_socket(self):
        peer=self.Peer()
        self.c.client=peer
        self.c.mode='monitor'
        self.c.pose_revision=12
        self.c.target=[-100]*6
        with patch('control.R3Client') as factory:
            self.c.check_connection()
            self.finish()
            factory.assert_not_called()
        self.assertEqual(peer.calls,['JPOSF'])
        self.assertEqual(self.c.connection_check['status'],'ok')
        self.assertEqual(self.c.pose_revision,13)
        self.assertEqual(self.c.target,peer.joints())

    def test_stale_connection_reopens_without_rearming(self):
        old=self.Peer(fail=True)
        new=self.Peer()
        self.c.client=old
        self.c.mode='monitor'
        self.c.armed=True
        self.c.stop()
        with patch('control.R3Client',return_value=new):
            self.c.check_connection()
            self.finish()
        self.assertIn('close',old.calls)
        self.assertEqual(new.calls,['connect','JPOSF'])
        self.assertIs(self.c.client,new)
        self.assertFalse(self.c.armed)

    def test_invalid_feedback_blocks_operation_and_closes_socket(self):
        peer=self.Peer(fail=True)
        with patch('control.R3Client',return_value=peer):
            self.c.check_connection()
            self.finish()
        self.assertEqual(peer.calls,['connect','JPOSF','close'])
        self.assertIsNone(self.c.client)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.connection_check['status'],'error')
        self.assertTrue(self.c.connection_check['tcp'])
        self.assertFalse(self.c.connection_check['position'])
        self.assertFalse(self.c.pose_synced)
        with self.assertRaises(ValueError): self.c.move([1,0,0,0,0,0])

    def test_check_rejected_during_motion(self):
        self.c.busy=True
        with self.assertRaises(ValueError): self.c.check_connection()

    def test_format_error_is_distinguished_from_network_failure(self):
        class BadFormat(self.Peer):
            def joints(self): raise ProtocolError('test unknown format')
        peer=BadFormat()
        with patch('control.R3Client',return_value=peer):
            self.c.check_connection()
            self.finish()
        self.assertIn('TCP-связь установлена',self.c.message)
        self.assertIn('формат',self.c.message)
        self.assertNotIn('питание',self.c.message)
        self.assertFalse(self.c.armed)

    def test_arm_reads_new_pose_and_invalidates_old_slider_baseline(self):
        self.c.heartbeat_at=time.monotonic()
        class Peer(self.Peer):
            def command(self,cmd): self.calls.append(cmd)
            def joints(self):
                self.calls.append('JPOSF')
                return [37.123456,-21.234567,82.345678,44.456789,-31.567891,179.678912]
        peer=Peer()
        self.c.client=peer
        self.c.mode='monitor'
        self.c.config.update(motion_commissioned=True,controller_firmware='test-only')
        with self.c.lock: self.c._synchronize_pose([0]*6)
        old_revision=self.c.pose_revision
        self.c.arm(True)
        self.assertEqual(self.c.pose_revision,old_revision+1)
        self.assertEqual(self.c.q,self.c.target)
        self.assertEqual(self.c.q,peer.joints())
        self.assertFalse(any('Mov' in cmd or 'JPCDX' in cmd for cmd in peer.calls))

    def test_missing_or_old_slider_baseline_never_sends_motion(self):
        peer=self.Peer()
        self.c.client=peer
        self.c.mode='monitor'
        self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        with self.c.lock: self.c._synchronize_pose([0]*6)
        for revision in (None,0,True,'1'):
            with self.subTest(revision=revision), self.assertRaisesRegex(ValueError,'синхронизированы'):
                self.c.move([1,0,0,0,0,0],revision)
        self.assertEqual(peer.calls,[])

    def test_incomplete_pose_never_becomes_synced(self):
        with self.c.lock:
            with self.assertRaises(ValueError): self.c._synchronize_pose([10,20,30])
        self.assertFalse(self.c.pose_synced)
        self.assertEqual(self.c.pose_revision,0)

    def test_reconnect_discards_previous_target_and_permission(self):
        old=self.Peer(fail=True)
        new=self.Peer()
        self.c.client=old
        self.c.mode='monitor'
        self.c.armed=True
        self.c.pose_revision=9
        self.c.target=[0]*6
        self.c.stop()
        with patch('control.R3Client',return_value=new):
            self.c.check_connection()
            self.finish()
        self.assertEqual(self.c.target,[5,0,90,0,0,0])
        self.assertEqual(self.c.pose_revision,10)
        self.assertFalse(self.c.armed)
        self.assertTrue(self.c.pose_synced)

    def test_current_slider_baseline_can_submit_only_a_bounded_step(self):
        ready_status(self.c)
        self.c.client=self.Peer()
        self.c.mode='monitor'
        self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        with self.c.lock: self.c._synchronize_pose([0]*6)
        with patch('control.threading.Thread') as worker:
            self.c.move([.1,0,0,0,0,0],self.c.pose_revision)
            worker.return_value.start.assert_called_once()
        self.assertTrue(self.c.busy)


class HomeTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(copy.deepcopy(load_config()))
        self.c.closed=True

    def tearDown(self):
        self.c.cancel.set()
        deadline=time.monotonic()+2
        while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
        if self.c.client: self.c.client.close()

    def finish(self):
        deadline=time.monotonic()+2
        while self.c.busy and time.monotonic()<deadline: time.sleep(.005)
        self.assertFalse(self.c.busy)

    def test_home_is_exact_recorded_pose_and_not_mutable_feedback(self):
        expected=tuple(OBSERVED_POSE)
        self.assertEqual(self.c.home_pose,expected)
        self.c.config['home_pose'][0]=100
        with self.c.lock: self.c._synchronize_pose([0]*6)
        self.assertEqual(self.c.home_pose,expected)
        published=self.c.snapshot()['home_pose']
        published[0]=-100
        self.assertEqual(self.c.home_pose,expected)
        restarted=Controller(load_config())
        restarted.closed=True
        self.assertEqual(restarted.home_pose,expected)

    def test_home_simulation_finishes_at_saved_pose(self):
        self.c.q=list(self.c.home_pose)
        self.c.q[0]+=.5
        self.c.home()
        self.finish()
        self.assertEqual(self.c.q,list(self.c.home_pose))
        self.assertIn('достигнута',self.c.message)

    def test_stop_cancels_home_without_changing_fixed_pose(self):
        saved=self.c.home_pose
        self.c.home()
        time.sleep(.05)
        self.c.stop()
        self.finish()
        stopped=self.c.q[:]
        time.sleep(.03)
        self.assertEqual(self.c.q,stopped)
        self.assertEqual(self.c.home_pose,saved)
        self.assertIn('отменён',self.c.message)

    def test_home_checks_session_freshness_and_enabled_setting(self):
        ready_status(self.c)
        peer=ConnectionCheckTests.Peer()
        self.c.client=peer
        self.c.mode='monitor'
        with self.assertRaisesRegex(ValueError,'не разрешено'): self.c.home()
        self.c.armed=True
        self.c.heartbeat_at=time.monotonic()
        with self.c.lock: self.c._synchronize_pose(list(self.c.home_pose))
        with self.assertRaisesRegex(ValueError,'синхронизированы'): self.c.home(0)
        self.c.config['home_motion_enabled']=False
        with self.assertRaisesRegex(ValueError,'отключён'): self.c.home(self.c.pose_revision)
        self.assertEqual(peer.calls,[])

    def test_home_cannot_be_queued_during_another_operation(self):
        self.c.busy=True
        with self.assertRaises(ValueError): self.c.home()
        self.c.busy=False

    def test_fake_robot_home_is_one_coordinated_move_at_controller_override(self):
        self.c.manual_step=2.
        ready_status(self.c)
        class Peer:
            def __init__(self,q): self.actual=q[:]; self.pending=None; self.moves=[]; self.commands=[]
            def joints(self): return self.actual[:]
            def status(self): return dict(parse_state(READY_STATE),override=37)
            def close(self): pass
            def command(self,cmd):
                self.commands.append(cmd)
                if cmd.startswith('EXECJPCDX='):
                    self.pending=[float(v) for v in cmd.split('(',1)[1][:-1].split(',')]
                if cmd=='EXECMov JPCDX':
                    self.moves.append((self.actual[:],self.pending[:]))
                    self.actual=self.pending[:]
        start=list(self.c.home_pose)
        start[0]+=3
        start[1]-=3
        start[2]+=2
        peer=Peer(start)
        self.c.client=peer
        self.c.mode='monitor'
        self.c.armed=True
        self.c.config.update(home_motion_enabled=True,controller_firmware='fake-only')
        self.c.heartbeat_at=time.monotonic()
        with self.c.lock: self.c._synchronize_pose(start)
        # No network or physical robot: make feedback sampling immediate in this test.
        with patch.object(self.c.cancel,'wait',side_effect=lambda delay:self.c.cancel.is_set()):
            self.c.home(self.c.pose_revision)
            self.finish()
        self.assertEqual(self.c.mode,'monitor')
        self.assertEqual(self.c.q,list(self.c.home_pose))
        self.assertEqual(len(peer.moves),1)
        self.assertFalse(any(cmd.startswith("OVRD=") for cmd in peer.commands))
        self.assertIn("EXECAccel 1,1",peer.commands)
        for before,after in peer.moves:
            delta=[abs(a-b) for a,b in zip(before,after)]
            self.assertEqual(sum(d>.001 for d in delta),3)
            self.assertEqual(after,list(self.c.home_pose))
        self.assertFalse(any('SRVON' in c for c in peer.commands))
        self.assertFalse(any(c.startswith(('EXECHOpen','EXECHClose')) for c in peer.commands))
        self.assertFalse(self.c.config['motion_commissioned'])
        self.assertFalse(self.c.config['home_motion_commissioned'])


if __name__ == '__main__': unittest.main()
