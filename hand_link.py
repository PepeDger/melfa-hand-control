"""Authenticated loopback transport for the existing camera/Mapper loop."""
import json
import base64
import sys
import time
import urllib.request
import uuid


class Link:
    def __init__(self, port=8765, dry=False):
        self.dry = dry
        credentials = {} if dry else json.loads(sys.stdin.readline())
        self.origin = f'http://127.0.0.1:{port}'
        self.token = credentials.get('token', '')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.session = str(uuid.uuid4())
        self.seq = self.action = 0
        self.mode = 'STOP'
        self.last_ack = self.last_send = -100.
        self.reply = None
        self.rtts = []
        self.ack_action = -1
        self.ack_command = None
        self.input_id = -1

    @property
    def connected(self):
        return self.dry or time.perf_counter()-self.last_ack < .3

    @property
    def calibrated(self):
        return self.dry or (self.mode == 'CALIBRATE' and
                           self.ack_action == self.action and self.ack_command == 'CALIBRATE')

    def send(self, command, pose=None, axis=-1, age_ms=0, axes=None):
        if command != self.mode:
            self.action += 1
        self.mode = command
        self.seq += 1
        self.last_send = time.perf_counter()
        if self.dry:
            return
        packet = dict(version=4, session=self.session, sequence=self.seq, action=self.action,
                      command=command, axis=axis, sentMs=int(time.time()*1000), ageMs=age_ms)
        if pose is not None:
            packet['pose'] = pose
        if axes is not None:
            packet['axes'] = axes
        request = urllib.request.Request(self.origin+'/api/hand-packet',
            data=json.dumps(packet, allow_nan=False).encode(), headers={
                'Content-Type': 'application/json', 'Origin': self.origin,
                'X-Session-Token': self.token})
        try:
            with self.opener.open(request, timeout=.2) as response:
                reply = json.load(response)
            if reply.get('session') != self.session or reply.get('sequence') != self.seq:
                return
            self.reply = reply
            self.last_ack = time.perf_counter()
            self.ack_action = reply['action']
            self.ack_command = reply['command']
            self.rtts.append((self.last_ack-self.last_send)*1000)
        except (OSError, ValueError, KeyError):
            self.ack_command = None

    def poll(self):
        if self.mode != 'TARGET' and time.perf_counter()-self.last_send > .1:
            self.send(self.mode)

    def take_key(self):
        if not self.reply or self.reply.get('inputId',-1) == self.input_id:
            return -1
        self.input_id=self.reply.get('inputId',-1)
        return {'SPACE':ord(' '),'CENTER':ord('c'),'QUIT':ord('q')}.get(self.reply.get('inputKey'),-1)

    def preview(self, jpeg):
        if self.dry:
            return
        request=urllib.request.Request(self.origin+'/api/hand-preview',
            data=json.dumps({'jpeg':base64.b64encode(jpeg).decode('ascii')}).encode(),
            headers={'Content-Type':'application/json','Origin':self.origin,'X-Session-Token':self.token})
        try:
            with self.opener.open(request,timeout=.2) as response: response.read()
        except (OSError,ValueError):
            pass

    def close(self):
        self.send('STOP')
