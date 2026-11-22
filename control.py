"""Bounded, single-flight MELFA R3 commissioning client. No auto servo-on."""
import json
import logging
import math
import re
import select
import socket
import threading
import time
import os
import tempfile
from pathlib import Path
from gripper import parse_hands, hands_frame_complete
from hand_settings import validate_hand_step
from diagnostics import parse_dstate, parse_state, status_frame_complete, movement_blockers

ROOT = Path(__file__).resolve().parent
DEFAULT_LIMITS_PATH = ROOT/'motion_limits.json'
LIMITS = ((-170, 170), (-90, 135), (-20, 171), (-160, 160), (-120, 120), (-360, 360))
MAX_STEP = 720.0
TRIAL_STEP = .1
HOME_TOLERANCE = .05
STEP_TOLERANCE = .01
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?"
JOINT_FIELDS = r'(?:J[1-8];' + NUMBER + r';){6,8}'
# Observed CR2D-721-S312 reply. Metadata is validated structurally, not treated
# as joint angles or interpreted as an alarm/safety/servo-ready indication.
SERVICE_FIELDS = (r';(?:\*{4}|[0-9A-Fa-f]{4}),(?:\*{4}|[0-9A-Fa-f]{4});'
                  + NUMBER + ';' + NUMBER + r';[0-9A-Fa-f]{8}')
EXTENDED_JPOSF = re.compile(JOINT_FIELDS + SERVICE_FIELDS)


class MotionBlocked(ValueError):
    """Valid telemetry currently forbids movement; the read-only link is usable."""


class ProtocolError(ValueError):
    """The peer replied, but the message did not match the expected protocol."""


def validate_speed(percent):
    if type(percent) is not int or not 1 <= percent <= 100:
        raise ValueError('Скорость контроллера должна быть целым числом от 1 до 100%')
    return percent


def validate_pose(values, limits=LIMITS):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("Нужно ровно шесть углов")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise ValueError("Углы должны быть числами")
    q = [float(v) for v in values]
    for i, (value, bounds) in enumerate(zip(q, limits)):
        if not math.isfinite(value) or not bounds[0] <= value <= bounds[1]:
            raise ValueError(f"J{i+1}: угол вне диапазона {bounds}")
    return q


def validate_motion_limits(values, physical=LIMITS):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError('Нужно задать минимум и максимум для всех шести осей')
    result = []
    for i, (bounds, allowed) in enumerate(zip(values, physical)):
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)):
            raise ValueError(f'J{i+1}: минимум и максимум должны быть конечными числами')
        lo, hi = map(float, bounds)
        if not allowed[0] <= lo < hi <= allowed[1]:
            raise ValueError(f'J{i+1}: требуется {allowed[0]:g}° ≤ минимум < максимум ≤ {allowed[1]:g}°')
        result.append([lo, hi])
    return result


def parse_joints(payload, reply_format='auto'):
    # Require explicit labels and field separators; never extract arbitrary numbers.
    pattern = r"J([1-8]);(" + NUMBER + r");"
    if reply_format not in ('auto', 'bare', 'cr2d_extended'):
        raise ValueError('Неизвестный профиль ответа JPOSF: ' + str(reply_format))
    extended = EXTENDED_JPOSF.fullmatch(payload)
    bare = re.fullmatch(JOINT_FIELDS, payload)
    if not ((extended and reply_format != 'bare') or (bare and reply_format != 'cr2d_extended')):
        raise ProtocolError('Неизвестный/неполный формат JPOSF: ' + repr(payload))
    joint_part = payload.split(';;', 1)[0] + ';' if extended else payload
    if extended and not all(math.isfinite(float(v)) for v in payload.split(';;', 1)[1].split(';')[1:3]):
        raise ProtocolError('Некорректные числовые служебные поля JPOSF')
    pairs = re.findall(pattern, joint_part)
    found = {}
    for axis, value in pairs:
        if axis in found:
            raise ProtocolError("Повтор оси в JPOSF")
        found[axis] = float(value)
    if not all(str(i) in found for i in range(1, 7)):
        raise ProtocolError("Неизвестный/неполный ответ JPOSF: " + repr(payload))
    return validate_pose([found[str(i)] for i in range(1, 7)])


class R3Client:
    """R3 wire profile must be qualified against the actual controller before motion.

    Legacy R3 does not always delimit replies. Without a configured terminator,
    collect until an idle gap, then validate the command-specific payload. This
    compatibility mode is NOT deterministic TCP framing or a safety mechanism.
    A timeout/parse error destroys the connection; commands are never retried.
    """
    def __init__(self, config):
        self.config = config
        if type(config.get('motion_commissioned')) is not bool:
            raise ValueError('motion_commissioned должен быть true или false')
        self.sock = None
        self.jposf_format = config.get('jposf_reply_format', 'auto')
        if self.jposf_format not in ('auto', 'bare', 'cr2d_extended'):
            raise ValueError('Неизвестный профиль ответа JPOSF')

    def connect(self):
        self.sock = socket.create_connection(
            (self.config['host'], int(self.config['port'])), self.config['timeout_s'])
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def command(self, command, data=False, slot=1):
        if type(slot) is not int or not 1 <= slot <= 32:
            raise ValueError('Неверный слот контроллера')
        if not self.sock:
            raise ConnectionError("Нет соединения")
        override_value = None
        if command.startswith('OVRD='):
            value = command[5:]
            if not re.fullmatch(r'[1-9][0-9]?|100', value):
                raise ValueError('OVRD: допустимы целые значения от 1 до 100')
            override_value = value
        try:
            if select.select([self.sock], [], [], 0)[0]:
                raise ConnectionError("Неожиданные данные до запроса; соединение закрыто")
            wire = ('1;' + str(slot) + ';' + command + self.config['request_terminator']).encode('ascii')
            logging.info('TX [%s:%s] %r', self.config['host'], self.config['port'], wire)
            self.sock.sendall(wire)
            deadline = time.monotonic() + self.config['timeout_s']
            result = bytearray()
            terminator = self.config['reply_terminator'].encode('ascii')
            extended_frame = command == 'JPOSF' and self.jposf_format == 'cr2d_extended' and not terminator
            status_frame = command in ('DSTATE', 'STATE') and not terminator
            hands_frame = command == 'HNDSTS' and not terminator
            # Observed CR2D-721: OVRD=1 returns QoK1 (the applied value).
            # This is command-specific; arbitrary payloads on ACKs stay invalid.
            override_echo = override_value is not None
            expected_override = ('QoK' + override_value).encode('ascii') if override_echo else None
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if result and (extended_frame or status_frame or hands_frame or override_echo):
                        raise ProtocolError('Неполный/неизвестный ответ ' + command + ': ' + repr(bytes(result)))
                    raise TimeoutError("Таймаут ответа; повторная отправка запрещена")
                wait = min(0.15, remaining) if result and not terminator else remaining
                if not select.select([self.sock], [], [], wait)[0]:
                    if result and not terminator:
                        if (extended_frame or status_frame or hands_frame or override_echo) and bytes(result).startswith(b'QoK'):
                            continue  # A pause in TCP is not the end of this known frame.
                        break
                    continue
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("Контроллер закрыл соединение")
                result.extend(chunk)
                if len(result) > 16384:
                    raise ValueError("Слишком длинный ответ")
                if override_echo and not terminator and bytes(result) == expected_override:
                    break
                if not data and not override_echo and not terminator and bytes(result) == b'QoK':
                    break  # Fixed three-byte acknowledgment, independent of TCP gaps.
                if hands_frame and bytes(result).startswith(b'QoK'):
                    if hands_frame_complete(bytes(result)[3:].decode('ascii').strip('\r\n\x00')): break
                if status_frame and bytes(result).startswith(b'QoK'):
                    if status_frame_complete(command, bytes(result)[3:].decode('ascii')):
                        break
                if extended_frame and bytes(result).startswith(b'QoK'):
                    body = bytes(result)[3:].decode('ascii').strip('\r\n\x00')
                    if EXTENDED_JPOSF.fullmatch(body):
                        break
                if terminator and terminator in result:
                    body, tail = bytes(result).split(terminator, 1)
                    if tail:
                        raise ValueError("Лишние данные после ответа")
                    result = bytearray(body)
                    break
            text = result.decode('ascii').strip('\r\n\x00')
            logging.info('RX [%s:%s] %r', self.config['host'], self.config['port'], text)
            if not text.startswith('QoK'):
                raise ProtocolError("Ошибка/неизвестный ответ контроллера: " + repr(text))
            payload = text[3:]
            if override_echo and payload != override_value:
                raise ProtocolError(command + ': ожидалось подтверждение применённого значения QoK' + override_value + ': ' + repr(text))
            if not data and not override_echo and payload:
                raise ValueError("Ожидалось подтверждение QoK: " + repr(text))
            return payload
        except Exception:
            self.close()
            raise

    def joints(self):
        try:
            return parse_joints(self.command('JPOSF', data=True), self.jposf_format)
        except Exception:
            self.close()
            raise

    def hands(self):
        try:
            return parse_hands(self.command('HNDSTS',data=True))
        except Exception:
            self.close()
            raise

    def status(self):
        # Read slot 1 plus global flags; never LOAD, reset, start or edit programs.
        try:
            detail = parse_state(self.command('STATE', data=True))
            current = parse_dstate(self.command('DSTATE', data=True))
            detail.update(current)  # The last global snapshot wins.
            return detail
        except Exception:
            self.close()
            raise


class Controller:
    def __init__(self, config, limits_path=None):
        self.config = config
        if type(config.get('motion_commissioned')) is not bool:
            raise ValueError('motion_commissioned должен быть true или false')
        self.limits = config['joint_limits']
        if type(config.get('physical_control_enabled', False)) is not bool:
            raise ValueError('physical_control_enabled должен быть true или false')
        manual_step = config.get('manual_step_deg', TRIAL_STEP)
        if (type(manual_step) not in (int, float) or not math.isfinite(manual_step) or
                not .01 <= manual_step <= MAX_STEP):
            raise ValueError('manual_step_deg должен быть числом от 0,01 до 720 градусов')
        self.manual_step = float(manual_step)
        joint_speed = config.get('joint_speed_percent', 10)
        if type(joint_speed) is not int or joint_speed not in (10, 20, 50, 100):
            raise ValueError('joint_speed_percent: допустимы 10, 20, 50 или 100')
        self.joint_speed = joint_speed
        self.override_percent = validate_speed(config.get('override_percent', 1))
        self.gripper_hand = config.get('gripper_hand')
        if self.gripper_hand is not None and (type(self.gripper_hand) is not int or not 1 <= self.gripper_hand <= 8):
            raise ValueError('gripper_hand должен быть номером 1–8 или null')
        self.gripper_channels = []
        self.gripper_at = 0.
        self.gripper_preview = False
        if len(self.limits) != 6:
            raise ValueError('Неверные ограничения осей')
        for configured, factory in zip(self.limits, LIMITS):
            if len(configured) != 2 or not all(math.isfinite(x) for x in configured) or not factory[0] <= configured[0] < configured[1] <= factory[1]:
                raise ValueError('Рабочие ограничения должны лежать внутри заводских')
        self.home_pose = tuple(validate_pose(config['home_pose'], self.limits))
        self.limits_path = Path(limits_path) if limits_path is not None else DEFAULT_LIMITS_PATH
        self.motion_limits = validate_motion_limits(self.limits, self.limits)
        if self.limits_path.exists():
            saved = json.loads(self.limits_path.read_text(encoding='utf-8'))
            if not isinstance(saved, dict) or saved.get('version') != 1:
                raise ValueError('Неверный формат motion_limits.json')
            self.motion_limits = validate_motion_limits(saved.get('limits'), self.limits)
        if type(config.get('home_motion_commissioned')) is not bool:
            raise ValueError('home_motion_commissioned должен быть true или false')
        if ('home_motion_enabled' in config and
                type(config['home_motion_enabled']) is not bool):
            raise ValueError('home_motion_enabled должен быть true или false')
        self.lock = threading.RLock()
        self.io = threading.Lock()
        self.cancel = threading.Event()
        self.client = None
        self.mode = 'simulation'
        self.q = [0., 0., 0., 0., 0., 0.]
        # Saved operator limits may exclude zero. This is the offline model
        # only; connecting always replaces it with measured robot joints.
        if any(not lo <= v <= hi for v,(lo,hi) in zip(self.q,self.motion_limits)):
            self.q = (list(self.home_pose) if all(lo <= v <= hi for v,(lo,hi) in zip(self.home_pose,self.motion_limits))
                      else [max(lo,min(hi,v)) for v,(lo,hi) in zip(self.q,self.motion_limits)])
        self.target = self.q[:]
        self.busy = False
        self.operation = None
        self.armed = False
        self.feedback_at = 0.
        self.heartbeat_at = 0.
        self.controller_status = None
        self.status_at = 0.
        self.diagnostic_error = ''
        self.operator_session = False
        self.program_context = None
        self.message = 'Симуляция. Соединения с роботом нет.'
        self.events = []
        self.pose_revision = 0
        self.pose_synced = False
        self.connection_check = dict(status='idle', tcp=False, position=False,
                                     detail='Проверка подключения ещё не выполнялась.')
        self.closed = False
        self.prepared_motion_controls = None
        self.hand_step_limit = .5
        threading.Thread(target=self._monitor, daemon=True).start()

    @property
    def max_step(self):
        return self.manual_step

    @property
    def range_description(self):
        if self.motion_limits != self.limits:
            return 'пользовательские границы осей'
        return ('заводские границы осей' if self.max_step >= max(b-a for a,b in self.limits)
                else f'шаг до {self.max_step:g}°')

    @property
    def home_enabled(self):
        # Explicit operator setting is separate from any claim of commissioning.
        return self.physical_enabled and self.config.get('home_motion_enabled',
            self.config['motion_commissioned'] and self.config['home_motion_commissioned'])

    @property
    def speed_commands(self):
        return (f'OVRD={self.override_percent}', f'EXECJOvrd {self.joint_speed}', 'EXECAccel 1,1')

    @property
    def physical_enabled(self):
        return bool((self.config.get('physical_control_enabled', False) or
                     self.config['motion_commissioned']) and self.config['controller_model'])

    def note(self, text):
        self.message = text
        self.events.append(time.strftime('%H:%M:%S') + ' ' + text)
        self.events = self.events[-40:]
        logging.info(text)

    def snapshot(self):
        with self.lock:
            return dict(mode=self.mode, joints=self.q[:], target=self.target[:], busy=self.busy,
                        armed=self.armed, message=self.message, limits=[x[:] for x in self.motion_limits],
                        physical_limits=[list(x) for x in self.limits],
                        range_description=self.range_description,
                        limit_violations=self._limit_violations(),
                        home_within_limits=all(lo <= v <= hi for v,(lo,hi) in zip(self.home_pose,self.motion_limits)),
                        feedback_age=None if not self.feedback_at else round(time.monotonic()-self.feedback_at, 2),
                        commissioned=bool(self.config['motion_commissioned']), events=self.events[-12:],
                        pose_revision=self.pose_revision, pose_synced=self.pose_synced,
                        connection_check=dict(self.connection_check),
                        profile_configured=self.physical_enabled, max_step=self.max_step,
                        full_joint_ranges=self.motion_limits == self.limits and self.max_step >= max(b-a for a,b in self.limits),
                        initial_control=not self.config['motion_commissioned'],
                        host=self.config['host'], port=self.config['port'], operation=self.operation,
                        home_pose=list(self.home_pose),
                        home_motion_commissioned=self.config['home_motion_commissioned'],
                        home_motion_enabled=self.home_enabled, home_motion_mode="coordinated",
                        joint_speed_percent=self.joint_speed,
                        speed_factor=self.override_percent * self.joint_speed / 10,
                        speed_percent=self.override_percent,
                        gripper=self._gripper_snapshot(),
                        controller_status=None if self.controller_status is None else dict(self.controller_status),
                        status_age=None if not self.status_at else round(time.monotonic()-self.status_at, 2),
                        movement_blockers=self._status_blockers(), diagnostic_error=self.diagnostic_error,
                        preparation_blockers=self._status_blockers(require_rights=False, operator_session=True),
                        operator_session=self.operator_session,
                        firmware=self.config['controller_firmware'])

    def _limit_violations(self):
        return [f'J{i+1}: текущее положение {v:.2f}° вне границ {lo:g}…{hi:g}°'
                for i,(v,(lo,hi)) in enumerate(zip(self.q,self.motion_limits)) if not lo <= v <= hi]

    def set_motion_limits(self, values):
        limits = validate_motion_limits(values, self.limits)
        with self.lock:
            if self.busy or self.armed:
                raise ValueError('Перед изменением границ нажмите STOP и дождитесь завершения операции')
            # Replace only a complete, valid file. Failure leaves active limits unchanged.
            temp = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.limits_path.parent,
                                                 prefix='.motion-limits-', suffix='.tmp', delete=False) as f:
                    temp = Path(f.name)
                    json.dump({'version':1, 'limits':limits}, f, ensure_ascii=False, allow_nan=False, indent=2)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(temp, self.limits_path)
            finally:
                if temp is not None and temp.exists(): temp.unlink()
            self.motion_limits = limits
            self.pose_revision += 1
            self.target = self.q[:]
            self.note('Границы J1–J6 сохранены. Команды контроллеру не отправлялись.' +
                      (' Текущее положение вне границ; движение заблокировано до изменения границ.'
                       if self._limit_violations() else ''))

    def _gripper_snapshot(self):
        selected = next((x for x in self.gripper_channels if x['hand']==self.gripper_hand),None)
        return dict(name='Z7',kind='binary',hand=self.gripper_hand,
                    closed=selected['closed'] if selected else None,
                    output=selected['output'] if selected else None,
                    solenoid_type=selected['solenoid_type'] if selected else None,
                    feedback_age=None if not self.gripper_at else round(time.monotonic()-self.gripper_at,2),
                    preview_closed=self.gripper_preview)

    def _read_gripper_status(self):
        channels = self.client.hands()
        with self.lock:
            self.gripper_channels = channels
            self.gripper_at = time.monotonic()
        selected = next((x for x in channels if x['hand']==self.gripper_hand),None)
        return selected

    def read_gripper(self, background=False):
        with self.lock:
            if self.mode!='monitor' or not self.client or self.busy:
                raise ValueError('Чтение захвата доступно после подключения, вне выполняющейся операции')
            self.busy=True
            self.operation='gripper_read'
        if background:
            threading.Thread(target=self._read_gripper_reserved, daemon=True).start()
            return
        return self._read_gripper_reserved()

    def _read_gripper_reserved(self):
        try:
            with self.io:
                selected=self._read_gripper_status()
            with self.lock:
                if not selected or selected['output']==-1:
                    self.note('Z7: выбранный канал захвата не настроен в контроллере.')
                else:
                    self.note(f"Z7: Hand {self.gripper_hand}, выход {selected['output']}; прочитан сигнал выхода, не датчики губок.")
        except Exception as exc:
            if self.armed: self.stop()
            with self.lock: self._fault('Чтение Z7: '+str(exc))
            raise
        finally:
            with self.lock: self.busy=False; self.operation=None

    def set_gripper(self, closed, pose_revision=None, background=False):
        if type(closed) is not bool:
            raise ValueError('Z7: допустимы только Открыть / Закрыть')
        with self.lock:
            self._check_motion_ready(pose_revision)
            if self.mode=='simulation':
                self.gripper_preview=closed
                self.note('Z7: '+('закрыть' if closed else 'открыть')+' — только симуляция.')
                return
            if self.gripper_hand is None:
                raise ValueError('Z7: номер канала захвата не настроен')
            self.busy=True
            self.operation='gripper'
            self.cancel.clear()
        if background:
            threading.Thread(target=self._set_gripper_reserved, args=(closed,), daemon=True).start()
            return
        return self._set_gripper_reserved(closed)

    def _set_gripper_reserved(self, closed):
        try:
            with self.io:
                self._command_guard()
                self._read_status(require_ready=True)
                selected=self._read_gripper_status()
                if not selected or selected['output']==-1:
                    raise MotionBlocked('Z7: выбранный Hand не назначен выходу контроллера')
                if selected['closed']!=closed:
                    command=f"EXEC{'HClose' if closed else 'HOpen'} {self.gripper_hand}"
                    self._execute_setup(command)
                    deadline=time.monotonic()+2
                    while True:
                        self._command_guard()
                        self._read_status(require_ready=True)
                        selected=self._read_gripper_status()
                        if selected and selected['output']!=-1 and selected['closed']==closed: break
                        if time.monotonic()>=deadline:
                            raise MotionBlocked('Z7: сигнал выхода не подтвердил команду; автоматического повтора нет')
                        if self.cancel.wait(.05): raise MotionBlocked('Команда Z7 отменена')
                self._command_guard()
                with self.lock:
                    self.gripper_preview=closed
                    self.note('Z7: сигнал '+('закрытия' if closed else 'открытия')+
                              f' Hand {self.gripper_hand} подтверждён. Положение губок датчиками не проверено.')
        except Exception as exc:
            self.stop()
            with self.lock: self._fault('Управление Z7: '+str(exc))
            raise
        finally:
            with self.lock: self.busy=False; self.operation=None

    @staticmethod
    def _context(status):
        return tuple(status.get(k) for k in ('program', 'line', 'task_program'))

    def _status_blockers(self, during_move=False, require_rights=True, operator_session=None, require_servo=True):
        reasons = movement_blockers(self.controller_status, during_move, require_rights,
                                    self.operator_session if operator_session is None else operator_session,
                                    require_servo)
        if (self.controller_status is not None and self.program_context is not None and
                self._context(self.controller_status) != self.program_context):
            reasons.insert(0, 'Выбранная программа или строка изменились; требуется новая подготовка')
        if self.status_at and time.monotonic()-self.status_at > 1.5:
            reasons.insert(0, 'Состояние контроллера устарело')
        if self.pose_synced and self.operation != 'servo':
            reasons.extend(self._limit_violations())
        return reasons

    def _read_status(self, during_move=False, require_ready=False, require_rights=True, require_servo=True):
        # Caller holds self.io. All failures keep physical control disabled.
        status = self.client.status()
        with self.lock:
            self.controller_status = status
            self.status_at = time.monotonic()
            self.diagnostic_error = ''
        with self.lock:
            reasons = self._status_blockers(during_move, require_rights, require_servo=require_servo)
        if require_ready and reasons:
            raise MotionBlocked('; '.join(reasons))
        return status

    def diagnose(self):
        with self.lock:
            if self.mode != 'monitor' or not self.client or self.busy or self.armed:
                raise ValueError('Диагностика доступна после подключения, без активного управления')
            self.busy = True
            self.operation = 'diagnostics'
        threading.Thread(target=self._diagnose, daemon=True).start()

    def _diagnose(self):
        try:
            with self.io:
                self._read_status()
            with self.lock:
                reasons = self._status_blockers()
                self.note('Диагностика без движения: ' + ('; '.join(reasons) if reasons else
                          'прочитанные признаки допускают движение; остальные слоты и профиль требуют проверки'))
        except Exception as exc:
            with self.lock:
                self.controller_status = None
                self.status_at = 0.
                self.diagnostic_error = str(exc)
                self._fault('Диагностика: ' + str(exc))
        finally:
            with self.lock:
                self.busy = False
                self.operation = None


    def heartbeat(self):
        with self.lock:
            self.heartbeat_at = time.monotonic()

    def _synchronize_pose(self, q):
        """Caller holds self.lock. Publish a complete actual pose as a new baseline."""
        q = validate_pose(q, self.limits)
        self.q = q[:]
        self.target = q[:]
        self.feedback_at = time.monotonic()
        self.pose_revision += 1
        self.pose_synced = True

    def connect(self):
        self.check_connection()

    def check_connection(self):
        with self.lock:
            if self.armed:
                raise ValueError('Перед повторной проверкой подключения нажмите STOP')
            if self.busy:
                raise ValueError('Дождитесь окончания текущей операции перед проверкой подключения')
            self.busy = True
            self.pose_synced = False
            self.controller_status = None
            self.status_at = 0.
            self.operator_session = False
            self.program_context = None
            self.cancel.clear()
            self.connection_check = dict(status='checking', tcp=False, position=False,
                                         detail='Проверяем соединение и читаем шесть осей…')
            self.note('Проверка подключения к ' + self.config['host'] + ':' + str(self.config['port']))
        threading.Thread(target=self._check_connection, daemon=True).start()

    def _check_connection(self):
        client = None
        tcp_ok = False
        try:
            with self.io:
                with self.lock:
                    client = self.client
                q = None
                if client:
                    try:
                        q = client.joints()
                        tcp_ok = True
                    except Exception:
                        # Only this explicit read-only check may reopen a stale socket.
                        # Motion/control commands are never retried or replayed.
                        client.close()
                        with self.lock:
                            self.client = None
                            self.armed = False
                            self.feedback_at = 0.
                        client = None
                if self.cancel.is_set():
                    return
                if client is None:
                    client = R3Client(self.config)
                    client.connect()
                    tcp_ok = True
                    with self.lock:
                        self.connection_check['tcp'] = True
                        self.connection_check['detail'] = 'TCP установлен. Проверяем ответ с положением осей…'
                    q = client.joints()  # Read-only: no CNTLON, SRVON, reset or MOVE.
                validate_pose(q, self.limits)
                with self.lock:
                    if self.cancel.is_set():
                        return
                    newly_connected = self.client is not client
                    self.client = client
                    if newly_connected:
                        self.armed = False
                    self.mode = 'monitor'
                    self._synchronize_pose(q)
                    detail = ('Связь с контроллером проверена, положение шести осей прочитано. '
                              'Цель синхронизирована с роботом. ')
                    if self.armed:
                        detail += 'Разрешение приложения на управление активно.'
                    elif self.physical_enabled:
                        detail += f'Доступна подготовка управления: согласованное движение осей, {self.range_description}.'
                    else:
                        detail += 'Движения заблокированы: профиль контроллера ещё не проверен.'
                    self.connection_check = dict(status='ok', tcp=True, position=True, detail=detail)
                    self.note(detail)
        except Exception as exc:
            with self.lock:
                self.armed = False
                self.client = None
                self.feedback_at = 0.
                self.pose_synced = False
                self.mode = 'fault'
                self.gripper_channels = []
                self.gripper_at = 0.
                if tcp_ok and isinstance(exc, ProtocolError):
                    hint = 'TCP-связь установлена, но формат ответа не прошёл проверку. Требуется проверка протокола.'
                elif tcp_ok and isinstance(exc, ValueError):
                    hint = 'TCP-связь установлена, но считанное положение не прошло проверку.'
                elif tcp_ok:
                    hint = ('TCP-соединение открыто, но ответ R3 не получен полностью. '
                            'Если RT Toolbox сейчас Online, переведите его в Offline и повторите подключение: '
                            'возможен конфликт сеансов на порту 10001.')
                else:
                    hint = 'TCP-соединение не установлено. Проверьте сеть и питание контроллера.'
                detail = ('Проверка не пройдена: ' + str(exc) + '. ' + hint +
                          ' Физическое управление заблокировано.')
                self.connection_check = dict(status='error', tcp=tcp_ok, position=False, detail=detail)
                self.note(detail)
        finally:
            with self.lock:
                if client is not None and client is not self.client:
                    client.close()
                if self.connection_check['status'] == 'checking':
                    self.connection_check = dict(status='cancelled', tcp=False, position=False,
                                                 detail='Проверка отменена. Готовность не подтверждена.')
                self.busy = False

    def arm(self, acknowledged):
        # Do not hold the state lock during network I/O: heartbeat and STOP must
        # remain responsive while the preparation request is being processed.
        with self.io:
            with self.lock:
                if self.mode != 'monitor' or not self.client or self.busy:
                    raise ValueError('Сначала подключитесь в режиме чтения')
                if not self.physical_enabled:
                    raise ValueError('Физическое управление отключено в настройках приложения')
                if acknowledged is not True:
                    raise ValueError('Подтвердите готовность зоны, останов других программ и аппаратного останова')
                if time.monotonic()-self.feedback_at > 1.5:
                    raise ValueError('Положение устарело')
                self.busy = True
                self.operation = 'arming'
                self.operator_session = True  # Existing operator acknowledgment; no automatic takeover.
                self.program_context = None
                self.cancel.clear()
                self.note('Подготовка: запрашиваем право управления и задаём ограничения. Движение не отправляется.')
            acquired = False
            try:
                initial = self._read_status(require_ready=True, require_rights=False)
                with self.lock:
                    self.program_context = self._context(initial)
                for cmd in ('CNTLON', *self.speed_commands):
                    if self.cancel.is_set():
                        raise MotionBlocked('Подготовка управления отменена')
                    if cmd.startswith('EXEC'):
                        self._execute_setup(cmd)
                    else:
                        self.client.command(cmd)
                    if cmd == 'CNTLON':
                        acquired = True
                        self._read_status(require_ready=True)
                status = self._read_status(require_ready=True)
                if status['override'] != self.override_percent:
                    raise MotionBlocked(f'Контроллер не подтвердил скорость OVRD={self.override_percent}')
                q = self.client.joints()
                with self.lock:
                    if self.cancel.is_set():
                        raise MotionBlocked('Подготовка управления отменена')
                    if time.monotonic()-self.heartbeat_at > 1:
                        raise MotionBlocked('Нет свежей связи с окном оператора')
                    self._synchronize_pose(q)
                    self.armed = True
                    self.prepared_motion_controls = (self.override_percent, self.joint_speed)
                    self.note(f'Управление разрешено. Согласованное движение осей, {self.range_description}. '
                              f'Скорость контроллера {self.override_percent}%, разгон прежний. '
                              'Серво доступно через SVO ON/OFF.')
            except Exception as exc:
                release_error = None
                if acquired:
                    try:
                        self.client.command('CNTLOFF')
                    except Exception as cleanup_exc:
                        release_error = str(cleanup_exc)
                with self.lock:
                    self.armed = False
                    self.operator_session = False
                    self.program_context = None
                    if release_error:
                        self._fault(f'{exc}. Не подтверждено снятие права управления: {release_error}')
                    elif isinstance(exc, MotionBlocked) and self.client:
                        self.note('Подготовка управления: ' + str(exc))
                    else:
                        self._fault(str(exc))
                raise
            finally:
                with self.lock:
                    self.busy = False
                    self.operation = None

    def _command_guard(self):
        """Caller holds self.io. Cancellation is checked after blocking I/O too."""
        with self.lock:
            if self.cancel.is_set():
                raise MotionBlocked('Операция отменена')
            if time.monotonic()-self.heartbeat_at > 1:
                raise MotionBlocked('Нет свежей связи с окном оператора')

    def _execute_setup(self, command):
        """Wait for an accepted non-motion EXEC to return to a stopped context.

        Never sends START/RUN/reset, and never treats ACK as motion completion.
        Caller holds self.io; program identity and other operational checks remain.
        """
        self._command_guard()
        self.client.command(command)
        deadline = time.monotonic()+3
        stable = 0
        while True:
            self._command_guard()
            status = self._read_status(during_move=True, require_ready=True)
            idle = (not status['running'] and not status.get('program_running', False) and
                    status.get('stopped', False))
            stable = stable+1 if idle else 0
            if stable >= 2:
                # Reapply the idle checks, including the conditional STOP+WAIT rule.
                with self.lock:
                    reasons = self._status_blockers()
                if reasons: raise MotionBlocked('; '.join(reasons))
                return
            if time.monotonic() >= deadline:
                raise MotionBlocked('Команда подготовки не завершилась за 3 с')
            if self.cancel.wait(.03):
                raise MotionBlocked('Операция отменена')

    def _check_motion_ready(self, pose_revision):
        """Caller holds self.lock. HOME has the same guards as manual moves."""
        if self.busy:
            raise ValueError('Движение/операция уже выполняется; очереди команд нет')
        if self._limit_violations():
            raise MotionBlocked('; '.join(self._limit_violations())+'; сначала расширьте границы')
        if self.mode != 'simulation':
            if not self.armed or not self.client:
                raise ValueError('Физическое управление не разрешено')
            if time.monotonic()-self.feedback_at > 1.5 or time.monotonic()-self.heartbeat_at > 1:
                raise ValueError('Нет свежего положения или связи с интерфейсом')
            if (not self.pose_synced or type(pose_revision) is not int or
                    pose_revision != self.pose_revision):
                raise ValueError('Ползунки не синхронизированы с текущим подключением. Обновите положение.')
            reasons = self._status_blockers()
            if reasons:
                raise MotionBlocked('; '.join(reasons))

    def home(self, pose_revision=None):
        with self.lock:
            self._check_motion_ready(pose_revision)
            if self.mode != 'simulation' and not self.home_enabled:
                raise ValueError('Физический HOME отключён в настройках приложения.')
            # This stored tuple is independent of feedback, sliders and reconnects.
            target = validate_pose(self.home_pose, self.motion_limits)
            self.target = target[:]
            self.busy = True
            self.operation = 'home'
            self.cancel.clear()
            self.note('HOME: согласованный возврат к сохранённой позе на текущей скорости контроллера.')
        threading.Thread(target=self._move, args=(target, True), daemon=True).start()

    def move(self, target, pose_revision=None, hand_step=False, hand_target=False):
        with self.lock:
            target = validate_pose(target, self.motion_limits)
            self._check_motion_ready(pose_revision)
            if hand_step and hand_target:
                raise ValueError('Неоднозначный режим команды кисти')
            if hand_step:
                delta=[abs(a-b) for a,b in zip(target,self.q)]
                if sum(d>.001 for d in delta)!=1 or max(delta)>validate_hand_step(self.hand_step_limit)+1e-6:
                    raise MotionBlocked(f'Кисть: допустима одна ось и шаг не более {self.hand_step_limit:g}°')
            if self.mode != 'simulation':
                deltas = [abs(a-b) for a,b in zip(target,self.q)]
                if not any(d > .001 for d in deltas) or max(deltas) > self.max_step + 1e-6:
                    raise ValueError(f'Измените хотя бы одну ось; отклонение каждой оси не больше {self.max_step:g}° и в заводских границах')
            self.target = target[:]
            self.busy = True
            self.operation = 'move'
            self.cancel.clear()
            if self.mode != 'simulation':
                changes = [f'J{i+1}: {a:.2f}° → {b:.2f}°' for i,(a,b) in enumerate(zip(self.q,target))
                           if abs(b-a) > .001]
                self.note('Согласованное перемещение: ' + '; '.join(changes) + '.')
        threading.Thread(target=self._move, args=(target,False,hand_step,hand_target), daemon=True).start()

    def set_speed(self, percent, pose_revision=None):
        """Stage while disarmed; apply with STATE readback in an idle own session.

        Never sends Mov, servo-on or START. Do not hold the state lock over I/O.
        Changing speed during any operation, including HOME, is rejected.
        """
        percent = validate_speed(percent)
        with self.lock:
            if self.busy:
                raise ValueError('Дождитесь завершения операции перед изменением скорости')
            if not self.armed:
                self.override_percent = percent
                self.note(f'Выбрана скорость {percent}% для следующей подготовки; контроллеру команды не отправлялись.')
                return
            self._check_motion_ready(pose_revision)
            self.busy = True
            self.operation = 'speed'
            self.cancel.clear()
        try:
            with self.io:
                self._command_guard()
                self._read_status(require_ready=True)
                self._command_guard()
                self.client.command(f'OVRD={percent}')
                status = self._read_status(require_ready=True)
                self._command_guard()
                if status['override'] != percent:
                    raise MotionBlocked(f'Контроллер не подтвердил скорость OVRD={percent}')
                with self.lock:
                    self.override_percent = percent
                    if self.prepared_motion_controls is not None:
                        self.prepared_motion_controls = (percent, self.joint_speed)
                    self.note(f'Скорость контроллера {percent}% подтверждена. Движение не запускалось.')
        except Exception as exc:
            self.stop()
            with self.lock:
                self._fault('Изменение скорости: ' + str(exc))
            raise
        finally:
            with self.lock:
                self.busy = False
                self.operation = None

    def set_servo(self, enabled, acknowledged):
        """Explicit idle-only power change. Never starts or resumes a program."""
        if type(enabled) is not bool:
            raise ValueError('Состояние серво должно быть true или false')
        with self.io:
            with self.lock:
                if self.busy or self.mode != 'monitor' or not self.client:
                    raise ValueError('Сначала подключитесь и дождитесь завершения операции')
                if not self.physical_enabled or acknowledged is not True:
                    raise ValueError('Подтвердите готовность зоны перед переключением серво')
                if time.monotonic()-self.feedback_at > 1.5 or time.monotonic()-self.heartbeat_at > 1:
                    raise ValueError('Нет свежего положения или связи с интерфейсом')
                was_armed = self.armed
                self.busy = True
                self.operation = 'servo'
                self.operator_session = True
                self.program_context = None
                self.cancel.clear()
            acquired = False
            try:
                try:
                    initial = self._read_status(require_ready=True, require_rights=was_armed, require_servo=False)
                    with self.lock: self.program_context = self._context(initial)
                    if initial['servo_on'] != enabled:
                        if not was_armed:
                            self._command_guard()
                            self.client.command('CNTLON')
                            acquired = True
                        self._read_status(require_ready=True, require_servo=False)
                        self._command_guard()
                        self.client.command('SRVON' if enabled else 'SRVOFF')
                        deadline = time.monotonic()+5
                        while True:
                            self._command_guard()
                            status = self._read_status(require_ready=True, require_servo=False)
                            if status['servo_on'] == enabled: break
                            if time.monotonic() >= deadline:
                                raise MotionBlocked('Контроллер не подтвердил состояние серво')
                            if self.cancel.wait(.05): raise MotionBlocked('Переключение серво отменено')
                    self._command_guard()
                    q = self.client.joints()
                    with self.lock:
                        self._synchronize_pose(q)
                        self.note(('SVO ON' if enabled else 'SVO OFF') +
                                  ' подтверждено контроллером. Движение не запускалось.')
                finally:
                    if acquired or (was_armed and not enabled):
                        self.client.command('CNTLOFF')
                    with self.lock:
                        if not was_armed or not enabled:
                            self.armed = False
                            self.operator_session = False
                            self.program_context = None
            except Exception as exc:
                with self.lock: self._fault('Переключение серво: ' + str(exc))
                raise
            finally:
                with self.lock:
                    self.busy = False
                    self.operation = None

    def _execute_real_step(self, target, coordinated_home=False, hand_step=False, hand_target=False):
        target = validate_pose(target, self.motion_limits)
        if hand_target and (hand_step or coordinated_home):
            raise MotionBlocked('Неоднозначный режим цели кисти')
        hand_motion=hand_step or hand_target
        with self.io:
            with self.lock:
                step_limit = self.max_step
                planned_start = self.q[:]
                if hand_step and (coordinated_home or
                        sum(abs(a-b)>.001 for a,b in zip(target,planned_start))!=1 or
                        max(abs(a-b) for a,b in zip(target,planned_start))>validate_hand_step(self.hand_step_limit)+1e-6):
                    raise MotionBlocked('Кисть: недопустимый шаг')
                if coordinated_home and (self.operation != 'home' or tuple(target) != self.home_pose):
                    raise MotionBlocked('Согласованный возврат разрешён только к сохранённой HOME')
                if self.cancel.is_set() or not self.armed:
                    return False
                if time.monotonic()-self.heartbeat_at > 1:
                    raise ValueError('Связь с окном оператора устарела')
            actual = validate_pose(self.client.joints(), self.limits)
            start_actual = actual[:]
            if max(abs(a-b) for a,b in zip(actual,planned_start)) > STEP_TOLERANCE + 1e-6:
                raise MotionBlocked('Робот сместился после задания цели; повторите синхронизацию')
            delta = [abs(a-b) for a,b in zip(actual,target)]
            if not coordinated_home and (max(delta) > step_limit + 1e-6):
                raise ValueError('Робот сместился после задания цели; повторите синхронизацию')
            initial_status = self._read_status(require_ready=True)
            # HOME preserves the current controller override; it is not replaced
            # by a stale local selection. Controller-side Mov plans one trajectory.
            expected_speed = validate_speed(initial_status['override']) if coordinated_home else self.override_percent
            commands = self.speed_commands[1:] if coordinated_home else self.speed_commands
            if (hand_motion and self.prepared_motion_controls == (self.override_percent,self.joint_speed)
                    and initial_status['override'] == self.override_percent):
                # Own armed session already confirmed these settings. Still read
                # STATE/DSTATE and actual joints immediately before each Mov.
                commands = ()
            for cmd in commands:
                if self.cancel.is_set(): return False
                if cmd.startswith('EXEC'):
                    self._execute_setup(cmd)
                else:
                    self.client.command(cmd)
            if self.cancel.is_set(): return False
            angles = ','.join(f'{v:.5f}' for v in target)
            self._execute_setup('EXECJPCDX=(' + angles + ')')
            if self.cancel.is_set(): return False
            # Check again after setup commands, immediately before EXEC Mov.
            status = self._read_status(require_ready=True)
            if status['override'] != expected_speed:
                raise MotionBlocked(f'Скорость контроллера изменилась; ожидалось OVRD={expected_speed}')
            # Setup may have taken time; verify the real pose again just before Mov.
            actual = validate_pose(self.client.joints(), self.limits)
            delta = [abs(a-b) for a,b in zip(actual,target)]
            if max(abs(a-b) for a,b in zip(actual,start_actual)) > STEP_TOLERANCE + 1e-6:
                raise MotionBlocked('Робот сместился во время подготовки движения; повторите синхронизацию')
            if not coordinated_home and (max(delta) > step_limit + 1e-6):
                raise MotionBlocked('Робот сместился во время подготовки; задайте цель заново')
            self._command_guard()
            self.client.command('EXECMov JPCDX')
        # Larger manual steps retain the same low speed and need more time.
        # Full-range travel at 1% can exceed the former 90-second step timeout.
        deadline = time.monotonic()+600
        stable = 0
        # Poll moving hardware at >=100 ms. Once it reports the target and
        # STOP, collect the remaining independent confirmations immediately.
        # All three target+STOP observations are still required.
        poll_delay = .1 if hand_motion else .15
        while not self.cancel.wait(poll_delay):
            with self.io:
                q = validate_pose(self.client.joints(), self.limits)
                status = self._read_status(during_move=True, require_ready=True)
            with self.lock:
                self.q = q
                self.feedback_at = time.monotonic()
            finished = (max(abs(a-b) for a,b in zip(q,target)) <= STEP_TOLERANCE and
                        not status['running'] and not status.get('program_running', False) and
                        status.get('stopped', False))
            stable = stable+1 if finished else 0
            if stable >= 3: return True
            if hand_motion:
                poll_delay = 0 if finished else .1
            if time.monotonic() > deadline:
                raise TimeoutError('Цель не достигнута за 10 минут')
        return False

    def _move(self, target, is_home=False, hand_step=False, hand_target=False):
        try:
            if self.mode == 'simulation':
                start = self.q[:]
                # Quintic interpolation: zero velocity/acceleration at both ends.
                duration = max(.5, max(abs(a-b) for a,b in zip(start,target)) * 1.875 / 15)
                t0 = time.monotonic()
                while not self.cancel.is_set():
                    t = min(1., (time.monotonic()-t0)/duration)
                    s = 10*t**3 - 15*t**4 + 6*t**5
                    with self.lock:
                        self.q = [a+(b-a)*s for a,b in zip(start,target)]
                    if t >= 1: break
                    time.sleep(.02)
            elif is_home:
                self._execute_real_step(target, coordinated_home=True)
            elif hand_step:
                self._execute_real_step(target, hand_step=True)
            elif hand_target:
                self._execute_real_step(target, hand_target=True)
            else:
                self._execute_real_step(target)
            with self.lock:
                if is_home and not self.cancel.is_set() and max(abs(a-b) for a,b in zip(self.q,target)) > HOME_TOLERANCE:
                    raise RuntimeError('HOME прерван до достижения сохранённой позы')
                self.target = self.q[:]
                if self.mode == 'monitor':
                    self.pose_revision += 1
                if self.cancel.is_set():
                    self.note('HOME отменён' if is_home else 'Движение отменено')
                else:
                    self.note('HOME: сохранённая поза достигнута' if is_home else 'Операция завершена')
        except Exception as exc:
            # A functioning link may still accept STOP after an operational fault.
            # If the network has failed, stop() reports that stopping is unconfirmed.
            if self.mode == 'monitor' and self.armed:
                self.stop()
            if isinstance(exc, MotionBlocked) and self.mode == 'monitor' and self.client:
                try:
                    with self.io:
                        q = self.client.joints()
                    with self.lock:
                        self._synchronize_pose(q)
                        self.note('Движение отменено: ' + str(exc) + '. Управление снято, доступно чтение.')
                except Exception as read_exc:
                    with self.lock: self._fault(str(exc) + '; ' + str(read_exc))
            else:
                with self.lock:
                    self._fault(str(exc))
        finally:
            with self.lock:
                self.busy = False
                self.operation = None

    def _fault(self, reason):
        self.prepared_motion_controls = None
        control_was_active = self.mode == 'monitor' and (self.armed or (self.busy and self.operation in ('move', 'home')))
        self.cancel.set()
        self.armed = False
        self.operator_session = False
        self.program_context = None
        self.pose_synced = False
        self.controller_status = None
        self.status_at = 0.
        self.pose_revision += 1
        if self.client: self.client.close()
        self.client = None
        self.mode = 'fault'
        self.gripper_channels = []
        self.gripper_at = 0.
        self.connection_check = dict(status='error', tcp=False, position=False,
                                     detail='Связь с контроллером потеряна: ' + reason)
        if control_was_active:
            detail = '. Остановка робота не подтверждена; используйте аппаратный STOP.'
        else:
            detail = '. Чтение положения остановлено. Физическое управление заблокировано.'
        self.note('СБОЙ: ' + reason + detail)

    def stop(self):
        self.prepared_motion_controls = None
        self.cancel.set()  # Set before waiting for network serialization.
        with self.lock:
            control_was_active = self.armed or (self.busy and self.operation in ('move', 'home'))
            self.armed = False
            self.operator_session = False
            self.program_context = None
            real = self.client is not None and control_was_active
        if real:
            try:
                with self.io:
                    if self.client:
                        self.client.command('STOP')
                        self.client.command('CNTLOFF')
                with self.lock:
                    self.note('STOP принят, право управления снято. Фактический останов проверьте на роботе.')
            except Exception as exc:
                with self.lock: self._fault(str(exc))
        else:
            with self.lock: self.note('Отмена локальной операции; команды роботу не отправлялись')

    def disconnect(self):
        self.stop()
        with self.io, self.lock:
            if self.busy:
                raise ValueError('Дождитесь завершения операции останова и повторите отключение')
            if self.client: self.client.close()
            self.client = None
            self.mode = 'simulation'
            self.gripper_channels = []
            self.gripper_at = 0.
            self.feedback_at = 0
            self.controller_status = None
            self.status_at = 0.
            self.operator_session = False
            self.program_context = None
            self.pose_synced = False
            self.pose_revision += 1
            self.target = self.q[:]
            self.connection_check = dict(status='idle', tcp=False, position=False,
                                         detail='Соединение отключено. Доступна симуляция.')
            self.note('Отключено. Симуляция.')

    def _monitor(self):
        while not self.closed:
            time.sleep(.25)
            if self.closed:
                break
            with self.lock:
                lost = self.armed and time.monotonic()-self.heartbeat_at > 1.5
                poll = self.client is not None and not self.busy
            if lost:
                self.stop()
                with self.lock: self.note('Связь с окном потеряна: управление снято, отправлен STOP. Проверьте останов.')
            if poll:
                try:
                    with self.io:
                        with self.lock:
                            if not self.client or self.busy: continue
                        q = validate_pose(self.client.joints(), self.limits)
                        with self.lock:
                            check_status = self.armed or self.controller_status is not None
                        with self.lock:
                            poll_hand=bool(self.gripper_channels and self.gripper_hand and time.monotonic()-self.gripper_at>.75)
                        if poll_hand: self._read_gripper_status()
                        if check_status:
                            self._read_status()
                            with self.lock:
                                blocked = self.armed and bool(self._status_blockers())
                            if blocked:
                                # Stop is issued outside the io lock below.
                                with self.lock: self.cancel.set()
                        with self.lock:
                            self.q = q
                            self.feedback_at = time.monotonic()
                            if not self.armed and not self.busy:
                                self.target = q[:]
                    if check_status and blocked:
                        self.stop()
                        with self.lock: self.note('Состояние изменилось: управление снято; проверьте аппаратный останов')
                except Exception as exc:
                    with self.lock: self._fault(str(exc))


def load_config():
    return json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
