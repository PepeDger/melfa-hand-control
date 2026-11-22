"""Local-only UI fixture. --ready enables a fake first-step control session.
Never connects to hardware. Listens on port 8766; manual steps remain bounded.
"""
from pathlib import Path
import re
import socketserver
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import app
from control import load_config

POSE=[1.53,-8.26,157.49,-.01,30.79,91.52]

class Robot(socketserver.BaseRequestHandler):
    def handle(self):
        pending=bytearray(); owned=False; override=100
        servo=getattr(self.server,'servo_on',True)
        waiting=getattr(self.server,'start_wait',False); busy_until=0.
        literals=('JPOSF','STATE','DSTATE','CNTLON','CNTLOFF',
                  f'EXECJOvrd {getattr(self.server,"joint_speed",10)}',
                  'EXECAccel 1,1','EXECMov JPCDX','STOP','SRVON','SRVOFF','HNDSTS')
        target=None
        while True:
            chunk=self.request.recv(4096)
            if not chunk: return
            pending.extend(chunk)
            while b'\n' in pending:
                body,_,tail=bytes(pending).partition(b'\n')
                pending=bytearray(tail)
                if not body.startswith(b'1;1;'):
                    self.request.sendall(b'QeR6010'); return
                cmd=body[4:].decode('ascii')
                if (cmd not in literals and not re.fullmatch(r'EXECJPCDX=\(([-+0-9.,]+)\)',cmd)
                        and not re.fullmatch(r'OVRD=(?:[1-9][0-9]?|100)',cmd)
                        and not re.fullmatch(r'EXECH(?:Open|Close) [1-8]',cmd)):
                    self.request.sendall(b'QeR6010'); return
                if hasattr(self.server,'commands'): self.server.commands.append(cmd)
                q=self.server.q
                flags=f'{1+(32 if servo and self.server.ready else 0)+(128 if owned else 0):02X}'
                running=time.monotonic()<busy_until
                if running: flags=f'{int(flags,16)|64:02X}'
                stop='00' if running else ('06' if waiting else '12')
                if cmd=='JPOSF':
                    reply='QoK'+''.join(f'J{i+1};{v:.5f};' for i,v in enumerate(q))+';****,****;100;0.00;00000000'
                elif cmd=='STATE': reply=f'QoKMAIN.MB5;1;{override};0;{flags}{stop}0;1;1;;;;;;;;;REP;START;1;0'
                elif cmd=='DSTATE': reply=f'QoK{flags}{stop}0;1;0'
                elif cmd=='HNDSTS':
                    reply='QoK'+f"900;{2 if getattr(self.server,'hand_closed',False) else 1};1;"+';'.join(['-1']*21)
                elif not self.server.ready: reply='QeR6010'
                else:
                    reply='QoK'
                    if cmd.startswith('EXEC') and running:
                        self.request.sendall(b'QeR6040'); continue
                    if cmd=='CNTLON': owned=True
                    if cmd=='CNTLOFF': owned=False
                    if cmd in ('SRVON','SRVOFF'):
                        if not owned:
                            self.request.sendall(b'QeR6010'); continue
                        servo=cmd=='SRVON'
                    if cmd.startswith('OVRD='):
                        override=int(cmd[5:]); reply=f'QoK{override}'
                    if cmd.startswith('EXEC'):
                        waiting=True
                        busy_until=time.monotonic()+getattr(self.server,'exec_delay',0.)
                    if cmd=='STOP': busy_until=0.; waiting=True
                    if cmd.startswith(('EXECHOpen ','EXECHClose ')):
                        if not owned or not servo or cmd.rsplit(' ',1)[1]!='1':
                            self.request.sendall(b'QeR6010'); continue
                        self.server.hand_closed=cmd.startswith('EXECHClose ')
                    if cmd.startswith('EXECJPCDX='): target=[float(v) for v in cmd.split('(',1)[1][:-1].split(',')]
                    if cmd=='EXECMov JPCDX':
                        delta=[] if target is None else [abs(a-b) for a,b in zip(q,target)]
                        if not owned or not servo or not 1<=override<=100 or len(delta)!=6 or max(delta)>getattr(self.server,'manual_step',.1)+1e-6:
                            reply='QeR6010'
                        else: self.server.q=target[:]
                self.request.sendall(reply.encode('ascii'))

if __name__=='__main__':
    robot=socketserver.ThreadingTCPServer(('127.0.0.1',0),Robot)
    robot.daemon_threads=True; robot.q=POSE[:]; robot.ready='--ready' in sys.argv
    robot.servo_on='--servo-off' not in sys.argv
    robot.start_wait=robot.ready; robot.exec_delay=.05
    robot.manual_step=load_config().get('manual_step_deg',.1)
    robot.joint_speed=load_config().get('joint_speed_percent',10)
    threading.Thread(target=robot.serve_forever,daemon=True).start()
    config=load_config()
    config.update(host='127.0.0.1',port=robot.server_address[1],motion_commissioned=False,
                  physical_control_enabled=robot.ready,auto_connect_read_only=False,
                  request_terminator='\n',reply_terminator='',jposf_reply_format='cr2d_extended')
    app.load_config=lambda:config
    print('LOCAL FAKE ROBOT ONLY; no hardware is used.',flush=True)
    try: app.serve(port=8766,open_browser=False)
    finally: robot.shutdown(); robot.server_close()
