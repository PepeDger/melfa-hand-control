"""Single-owner hand input: latest target only, independent tracking watchdog.

Camera recognition is untrusted input. Real moves use the existing controller
checks and one coordinated target, never a queue of camera frames.
The optional legacy mode still uses bounded single-axis segments.
"""
import json
import base64
import math
from pathlib import Path
import subprocess
import threading
import time

from hand_tracking.direct_core import FIELDS
from hand_settings import validate_sensitivity, validate_hand_step


class HandControl:
    TIMEOUT = .3
    BATCH_WINDOW = .02
    REAL_STEP = .5
    REAL_SPEED = 5.
    MODEL_SPEED = 30.
    MODEL_ACCEL = 100.

    def __init__(self, controller, start_worker=True, whole_target=True):
        self.c = controller
        self.lock = threading.RLock()
        self.dispatch = threading.RLock()
        self.enabled = self.active = self.real = self.grip_enabled = False
        self.closed = False
        self.session = None
        self.sequence = -1
        self.action = -1
        self.seen_stop = False
        self.last_packet = 0.
        self.target = None
        self.whole_target = whole_target
        self.multi_axis = False
        self.axes = []
        self.pending_since = None
        self.axis = -1
        self.velocity = [0.]*6
        self.status = 'Выключено'
        self.home_requested = False
        self.grip_class = self.grip_candidate = self.grip_pending = None
        self.grip_since = 0.
        self.sensitivity = 1.
        self.real_step = self.REAL_STEP
        self.grip_sync_requested = False
        self.grip_ready = False
        self.pinch_value = None
        self.grip_status = 'Щипок Z7 выключен'
        self.grip_requested = None
        self.process = None
        self.camera_exit_seen = False
        self.input_id = 0
        self.input_key = ''
        self.preview_jpeg = None
        self.preview_at = 0.
        self.last_tick = time.monotonic()
        self.last_move = 0.
        if start_worker:
            threading.Thread(target=self._worker, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return dict(enabled=self.enabled, active=self.active, real=self.real,
                        gripper_enabled=self.grip_enabled, axis=self.axis, status=self.status,
                        age=None if not self.last_packet else round(time.monotonic()-self.last_packet, 2),
                        camera_running=self.process is not None and self.process.poll() is None,
                        home_requested=self.home_requested, real_step_deg=self.real_step,
                        real_speed_deg_s=self.REAL_SPEED,
                        sensitivity=self.sensitivity, gripper_ready=self.grip_ready,
                        whole_target=self.whole_target, multi_axis=self.multi_axis, axes=self.axes[:],
                        goal=None if self.target is None else self.target[:], batch_ms=20,
                        pinch_percent=None if self.pinch_value is None else round(self.pinch_value*100),
                        gripper_status=self.grip_status,
                        preview_age=None if not self.preview_at else round(time.monotonic()-self.preview_at,2))

    def set_options(self, sensitivity=None, gripper=None, step_deg=None, whole_target=None, multi_axis=None):
        gain = None if sensitivity is None else validate_sensitivity(sensitivity)
        step = None if step_deg is None else validate_hand_step(step_deg)
        if gripper is not None and type(gripper) is not bool:
            raise ValueError('Z7: неверное значение флажка')
        if any(x is not None and type(x) is not bool for x in (whole_target,multi_axis)):
            raise ValueError('Неверный режим цели кисти')
        with self.dispatch, self.lock, self.c.lock:
            new_whole=self.whole_target if whole_target is None else whole_target
            new_multi=self.multi_axis if multi_axis is None else multi_axis
            if new_multi and not new_whole:
                raise ValueError('Несколько осей доступны в режиме «Цель целиком»')
            changed=(new_whole!=self.whole_target or new_multi!=self.multi_axis or
                     (step is not None and step!=self.real_step))
            if changed and (self.active or self.c.busy):
                raise ValueError('Для изменения шага или режима поставьте управление кистью на паузу')
            if gain is not None:self.sensitivity=gain
            if step is not None:self.real_step=step;self.c.hand_step_limit=step
            self.whole_target=new_whole;self.multi_axis=new_multi
            if changed:self.target=None;self.pending_since=None
            if gripper is not None:
                self._configure_gripper(gripper)

    def _configure_gripper(self, enabled):
        self.grip_enabled=enabled
        self.grip_class=self.grip_candidate=self.grip_pending=None
        self.grip_requested=None
        self.pinch_value=None
        self.grip_ready=enabled and not self.real
        self.grip_sync_requested=enabled and self.real
        self.grip_status=('Z7: читаем состояние Hand 1; затем разведите пальцы' if self.grip_sync_requested else
                          'Z7: разведите большой и указательный пальцы' if enabled else 'Щипок Z7 выключен')

    def camera_input(self, key):
        if key not in ('SPACE', 'CENTER', 'QUIT'):
            raise ValueError('Неверная кнопка камеры')
        with self.lock:
            if key != 'QUIT' and not self.enabled:
                raise ValueError('Сначала включите режим кисти')
            self.input_id += 1
            self.input_key = key

    def camera_preview(self, jpeg):
        if not isinstance(jpeg,str) or len(jpeg)>170000:
            raise ValueError('Неверный размер изображения камеры')
        data=base64.b64decode(jpeg,validate=True)
        if not data.startswith(bytes([255,216])) or not data.endswith(bytes([255,217])):
            raise ValueError('Неверное изображение камеры')
        with self.lock:
            self.preview_jpeg=data
            self.preview_at=time.monotonic()
            if self.status.startswith('Камера запускается'):
                self.status='Камера готова. Покажите одну кисть и нажмите «Калибровать / включить»'

    def enable(self, real=False, gripper=False, acknowledged=False):
        if any(type(v) is not bool for v in (real, gripper, acknowledged)):
            raise ValueError('Неверные параметры режима кисти')
        # dispatch also serializes STOP and the final move submission.
        with self.dispatch, self.lock, self.c.lock:
            if self.enabled:
                raise ValueError('Сначала выключите текущий режим кисти')
            if self.c.busy:
                raise ValueError('Дождитесь завершения операции')
            if real:
                if not acknowledged or self.c.mode != 'monitor':
                    raise ValueError('Подключите робота и подтвердите готовность рабочей зоны')
                self.c._check_motion_ready(self.c.pose_revision)
            elif self.c.mode != 'simulation':
                raise ValueError('Для управления только моделью сначала отключите реального робота')
            self.enabled = True
            self.real = real
            self._configure_gripper(gripper)
            self.active = False
            self.session = None
            self.sequence = self.action = -1
            self.seen_stop = False
            self.target = None
            self.pending_since = None
            self.axes = []
            self.home_requested = False
            self.status = 'Покажите кисть и нажмите «Калибровать / включить»'
            self.input_key = ''
            self.input_id += 1

    def pause(self, reason='Пауза; нажмите «Калибровать / включить» для продолжения', disable=False):
        with self.dispatch:
            with self.lock:
                owned = self.active
                self.active = False
                self.target = None
                self.pending_since = None
                self.axes = []
                self.axis = -1
                self.velocity = [0.]*6
                self.grip_pending = None
                self.input_key = ''
                self.input_id += 1
                self.seen_stop = False
                self.status = reason
                if disable:
                    self.enabled = False
                # Cancellation is visible before waiting for socket serialization.
                if owned:
                    self.c.cancel.set()
            if owned:
                self.c.stop()

    def ensure_manual(self):
        with self.lock:
            if self.enabled:
                raise ValueError('Выключите режим кисти перед ручной командой')

    def packet(self, p, now=None):
        now = time.monotonic() if now is None else now
        if (p.get('version') != 4 or not isinstance(p.get('session'), str) or
                not 1 <= len(p['session']) <= 80 or type(p.get('sequence')) is not int or
                type(p.get('action')) is not int or p['sequence'] < 1 or p['action'] < 0 or
                p.get('command') not in ('STOP', 'CALIBRATE', 'TARGET', 'HOME')):
            raise ValueError('Неверный пакет кисти')
        command = p['command']
        age = p.get('ageMs', 0)
        sent = p.get('sentMs')
        transit = time.time()*1000-sent if type(sent) is int else float('inf')
        if (type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 150 or
                type(sent) is not int or not -50 <= transit <= 250 or age+max(0,transit) > 250):
            self.pause('Устаревший кадр: STOP; повторите калибровку')
            raise ValueError('Кадр кисти устарел')
        with self.dispatch, self.lock:
            if not self.enabled:
                return self._reply(p, 'EXIT', 'Режим кисти выключен в веб-пульте')
            if self.session != p['session']:
                if self.session is not None or command != 'STOP':
                    raise ValueError('Другой сеанс камеры; выключите и включите режим кисти')
                self.session = p['session']
            if p['sequence'] <= self.sequence or p['action'] < self.action:
                raise ValueError('Повторный или старый пакет кисти')
            self.sequence = p['sequence']
            self.action = p['action']
            if self.active and now-self.last_packet > self.TIMEOUT:
                self.pause('Потеря кисти >300 мс: STOP; повторите подготовку и калибровку')
                return self._reply(p, 'STOP')
            if command == 'STOP':
                if self.active:
                    self.pause()
                self.seen_stop = True
                return self._reply(p, 'STOP')
            if command == 'CALIBRATE':
                if self.active:
                    # Repeated acknowledgements are idempotent, not a new origin.
                    return self._reply(p, 'CALIBRATE')
                if not self.seen_stop:
                    return self._reply(p, 'STOP', 'Сначала STOP, затем новая калибровка')
                with self.c.lock:
                    if self.c.mode != ('monitor' if self.real else 'simulation'):
                        return self._reply(p, 'STOP', 'Режим подключения изменился')
                    self.c._check_motion_ready(self.c.pose_revision)
                    self.c.cancel.clear()
                self.active = True
                self.seen_stop = False
                self.target = self.c.q[:] if self.whole_target else None
                self.pending_since = None
                self.axes = []
                self.grip_class = self.grip_candidate = self.grip_pending = None
                self.velocity = [0.]*6
                self.last_tick = self.last_move = self.last_packet = now
                self.status = 'Калибровка выполнена; ожидание движения кисти'
                return self._reply(p, 'CALIBRATE')
            if not self.active:
                return self._reply(p, 'STOP', 'STOP; повторите подготовку управления и калибровку')
            if command == 'HOME':
                self.pause('HOME запрошен: выключите режим кисти и нажмите HOME в веб-пульте')
                self.home_requested = True
                return self._reply(p, 'HOME')
            pose = p.get('pose')
            axis = p.get('axis')
            channels=p.get('axes')
            if channels is None:
                channels=[axis] if type(axis) is int and 0<=axis<=5 else []
            if (not isinstance(channels,list) or len(channels)>6 or
                    any(type(i) is not int or not 0<=i<=5 for i in channels) or
                    len(set(channels))!=len(channels) or (len(channels)>1 and not self.multi_axis)):
                self.pause('Неверные оси цели кисти: STOP')
                raise ValueError('Неверный набор осей кисти')
            if (not isinstance(pose, list) or len(pose) != 7 or
                    not all(type(v) in (int, float) and math.isfinite(v) for v in pose) or
                    type(axis) is not int or not -1 <= axis <= 5 or not 0 <= pose[6] <= 1 or
                    any(not lo <= v <= hi for v, (lo, hi) in zip(pose[:6], self.c.limits))):
                self.pause('Неверная цель кисти: STOP')
                raise ValueError('Неверная цель кисти')
            if axis != self.axis:
                if not self.whole_target:self.velocity = [0.]*6
            self.axes=channels[:]
            self.axis = channels[0] if len(channels)==1 else -1
            limited = False
            if self.whole_target:
                if self.target is None:self.target=self.c.q[:]
                for i in channels:
                    lo,hi=self.c.motion_limits[i];v=max(lo,min(hi,pose[i]))
                    limited=limited or v!=pose[i]
                    if abs(v-self.target[i])>1e-9 and self.pending_since is None:self.pending_since=now
                    self.target[i]=v
            else:
                self.target = [max(lo,min(hi,v)) for v,(lo,hi) in zip(pose[:6],self.c.motion_limits)]
                limited = axis >= 0 and self.target[axis] != pose[axis]
            self.last_packet = now-(age+max(0,transit))/1000
            self._pinch(pose[6], now)
            names=', '.join('J'+str(i+1) for i in channels)
            self.status = (names+': достигнута заданная граница' if limited else
                           'Кисть → '+names if channels else 'Удержание последней цели' if self.whole_target else 'Удержание положения')
            # Mapper drops any lagging accumulated target instead of replaying it.
            with self.c.lock:
                lag = not self.whole_target and axis >= 0 and abs(pose[axis]-self.c.q[axis]) > (self.real_step if self.real else 3.)
            return self._reply(p, 'TARGET', 'Direct blocked / configured limit' if limited else
                               'Direct coalesced / latest target' if lag else self.status)

    def _pinch(self, grip, now):
        self.pinch_value=grip
        if not self.grip_enabled or not self.grip_ready:
            return
        candidate = True if grip <= .25 else False if grip >= .75 else None
        if candidate is None:
            self.grip_candidate = None
            return
        if self.grip_candidate != candidate:
            self.grip_candidate = candidate
            self.grip_since = now
        if now-self.grip_since < .25:
            return
        if self.grip_class is None:
            # Require an open-hand baseline. Calibration never releases payload.
            if candidate:
                self.grip_status='Z7: сначала разведите большой и указательный пальцы'
                return
            self.grip_class = False
            self.grip_status='Z7 готов: сведите пальцы для закрытия'
        elif self.grip_class != candidate:
            self.grip_class = candidate
            if self.grip_enabled:
                self.grip_pending = candidate
                self.grip_status='Z7: команда '+('закрыть' if candidate else 'открыть')+' ожидает остановки оси'

    def _reply(self, p, command, status=None):
        with self.c.lock:
            in_flight=(self.real and self.c.busy and self.c.operation=='move' and not self.c.cancel.is_set())
            reference=(self.target if self.whole_target and self.target is not None else
                       self.c.target if in_flight else self.c.q)
            return dict(version=4, session=p['session'], sequence=p['sequence'],
                        action=p['action'], command=command, pose=dict(zip(FIELDS, self.c.q)),
                        hand_reference=dict(zip(FIELDS,reference)),
                        ikStatus=status or self.status, inputId=self.input_id, inputKey=self.input_key,
                        sensitivity=self.sensitivity, whole_target=self.whole_target, multi_axis=self.multi_axis)

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        with self.dispatch, self.lock:
            dt = max(.001, min(.05, now-self.last_tick))
            self.last_tick = now
            if not self.enabled:
                return
            if self.grip_sync_requested:
                with self.c.lock:
                    if self.c.mode != ('monitor' if self.real else 'simulation'):
                        self.grip_sync_requested=False;self.grip_status='Z7: режим подключения изменился'
                    elif not self.c.busy:
                        self.grip_sync_requested=False
                        if self.real: self.c.read_gripper(background=True)
            if self.grip_enabled and self.real and not self.grip_ready:
                with self.c.lock:
                    hand=self.c._gripper_snapshot()
                    if not self.c.busy and hand['feedback_age'] is not None:
                        if hand['output']==-1 or hand['closed'] is None:
                            self.grip_status='Z7: Hand 1 не назначен выходу контроллера'
                        elif hand['feedback_age']<1.5:
                            self.grip_ready=True
                            self.grip_status='Z7 прочитан. Разведите большой и указательный пальцы'
            if self.grip_requested is not None:
                with self.c.lock:
                    hand=self.c._gripper_snapshot()
                    actual=hand['closed'] if self.real else hand['preview_closed']
                    if not self.c.busy and actual==self.grip_requested:
                        self.grip_status='Z7: '+('закрытие' if actual else 'открытие')+' подтверждено сигналом выхода' if self.real else 'Z7: '+('модель закрыта' if actual else 'модель открыта')
                        self.grip_requested=None
            if not self.active:
                return
            with self.c.lock:
                invalid = (self.c.mode != ('monitor' if self.real else 'simulation') or
                           self.c.cancel.is_set() or (self.real and not self.c.armed))
                if invalid or now-self.last_packet > self.TIMEOUT:
                    # STOP I/O must not hold the controller state lock.
                    reason = 'Управление остановлено' if invalid else 'Потеря кисти >300 мс: STOP'
                else:
                    reason = None
            if reason:
                self.pause(reason+'; повторите подготовку и калибровку')
                return
            with self.c.lock:
                if self.c.busy:
                    return
                if self.grip_pending is not None:
                    closed = self.grip_pending
                    self.grip_pending = None
                    revision = self.c.pose_revision
                    # Actual I/O is run outside both state locks.
                    operation = ('gripper', closed, revision)
                elif self.whole_target:
                    if self.target is None:return
                    if self.real:
                        if self.pending_since is None or now-self.pending_since<self.BATCH_WINDOW:return
                        if max(abs(a-b) for a,b in zip(self.target,self.c.q))<.02:return
                        operation=('whole',self.target[:],self.c.pose_revision)
                        self.pending_since=None
                        self.last_move=now
                    else:
                        for i in range(6):
                            delta=self.target[i]-self.c.q[i]
                            desired=math.copysign(min(self.MODEL_SPEED,math.sqrt(2*self.MODEL_ACCEL*abs(delta))),delta)
                            v=self.velocity[i]+max(-self.MODEL_ACCEL*dt,min(self.MODEL_ACCEL*dt,desired-self.velocity[i]))
                            step=v*dt
                            if abs(step)>abs(delta) or step*delta<0:
                                step=delta if step*delta>=0 else 0;v=0.
                            self.velocity[i]=v;self.c.q[i]+=step
                        self.c.target=self.c.q[:];self.c.pose_revision+=1
                        return
                elif self.axis < 0 or self.target is None:
                    return
                elif self.real:
                    axis = self.axis
                    q = self.c.q[:]
                    delta = self.target[axis]-q[axis]
                    step = min(self.real_step, self.REAL_SPEED*min(1., now-self.last_move))
                    if abs(delta) < .02 or step < .02:
                        return
                    q[axis] += max(-step, min(step, delta))
                    operation = ('move', q, self.c.pose_revision)
                    self.last_move = now
                else:
                    i = self.axis
                    delta = self.target[i]-self.c.q[i]
                    desired = math.copysign(min(self.MODEL_SPEED, math.sqrt(2*self.MODEL_ACCEL*abs(delta))), delta)
                    v = self.velocity[i]
                    v += max(-self.MODEL_ACCEL*dt, min(self.MODEL_ACCEL*dt, desired-v))
                    step = v*dt
                    if abs(step) > abs(delta) or step*delta < 0:
                        step = delta if step*delta >= 0 else 0
                        v = 0.
                    self.velocity[i] = v
                    self.c.q[i] += step
                    self.c.target = self.c.q[:]
                    self.c.pose_revision += 1
                    return
            if operation[0] == 'whole':
                self.c.move(operation[1],operation[2],hand_target=True)
            elif operation[0] == 'move':
                self.c.move(operation[1], operation[2], hand_step=True)
            else:
                self.c.set_gripper(operation[1], operation[2], background=True)
                self.grip_requested=operation[1]
                self.grip_status='Z7: '+('закрыть' if operation[1] else 'открыть')+' отправлено; проверяется сигнал выхода'

    def _worker(self):
        while not self.closed:
            try:
                self.tick()
                with self.lock:
                    exited = self.process is not None and self.process.poll() is not None and not self.camera_exit_seen
                    if exited:
                        self.camera_exit_seen = True
                if exited:
                    self.pause('Камера остановлена. При ошибке смотрите hand-camera.log; откройте камеру снова')
            except Exception as exc:
                self.pause('Кисть остановлена: '+str(exc))
            time.sleep(.02)

    def launch_camera(self, port, token, camera=0, sensitivity=1.):
        if type(camera) is not int or not 0 <= camera <= 10:
            raise ValueError('Камера: номер 0–10')
        sensitivity=validate_sensitivity(sensitivity)
        root = Path(__file__).resolve().parent
        with self.lock:
            if not self.enabled:
                raise ValueError('Сначала включите режим кисти')
            if self.process is not None and self.process.poll() is None:
                raise ValueError('Окно камеры уже запущено')
            if self.active:
                raise ValueError('Сначала остановите предыдущее управление кистью')
            self.sensitivity=sensitivity
            candidates = [root/'.venv_hand/Scripts/python.exe', root.parent/'HandControl/.venv/Scripts/python.exe']
            python = next((p for p in candidates if p.is_file()), None)
            if python is None:
                raise ValueError('Запустите SetupHandTracking.cmd для установки распознавания кисти')
            if not (root/'hand_landmarker.task').is_file():
                raise ValueError('Не найден hand_landmarker.task; восстановите файл из архива')
            self.session = None
            self.sequence = self.action = -1
            self.seen_stop = False
            self.input_key = ''
            self.input_id += 1
            self.preview_jpeg = None
            self.preview_at = 0.
            log = (root/'hand-camera.log').open('a', encoding='utf-8')
            try:
                self.process = subprocess.Popen([str(python), str(root/'hand_camera.py'),
                    '--port', str(port), '--camera', str(camera), '--sensitivity', str(sensitivity), '--web-ui'],
                    cwd=root, stdin=subprocess.PIPE, stdout=log, stderr=log,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                self.process.stdin.write(json.dumps({'token':token}).encode() + bytes([10]))
                self.process.stdin.close()
            finally:
                log.close()
            self.status = 'Камера запускается; покажите кисть и нажмите «Калибровать / включить»'
            self.camera_exit_seen = False
