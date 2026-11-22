"""Direct six-axis hand control, one dominant movement at a time. No IK solving."""
import sys
import math
import time
import socket
import uuid
import json
from pathlib import Path
from .unified_core import sample, LatestCamera, UnstableHand, qangle, qlerp, qmul, inverse, clamp
from hand_settings import validate_sensitivity
FIELDS=('baseAngle','shoulderAngle','elbowAngle','forearmAngle','wristAngle','toolAngle')
LIMITS=((-170,170),(-90,135),(-20,171),(-160,160),(-120,120),(-360,360))
HOME=dict(zip(FIELDS,[0,-15,50,0,55,0]))
LABELS=('J1 BASE / left-right','J2 SHOULDER / up-down','J3 ELBOW / near-far','J4 TWIST / palm turn','J5 PITCH / palm tilt','J6 ROLL / wrist roll')
class Mapper:
 def __init__(self,sensitivity=1):
  self.sensitivity=validate_sensitivity(sensitivity);self.active=None;self.candidate=None;self.candidate_time=0;self.quiet=None;self.active_label='HOLD'
 def calibrate(self,f,pose,now):
  self.actual=[float(pose[k]) for k in FIELDS];self.goal=self.actual[:]
  self.reference=self.actual[:];self.whole_target=False
  self.last=dict(f);self.filtered=dict(f);self.last_time=now
  self.active=None;self.candidate=None;self.quiet=None;self.active_label='HOLD';self.scores=[0]*6
  self.drive_direction=0;self.reverse_direction=0;self.reverse_since=now
 def reconcile(self,reply):
  if not reply or 'pose' not in reply:return
  self.actual=[float(reply['pose'][k]) for k in FIELDS]
  self.whole_target=reply.get('whole_target') is True
  reference=reply.get('hand_reference',reply['pose'])
  values=[reference.get(k) for k in FIELDS] if isinstance(reference,dict) else []
  valid=len(values)==6 and all(type(v) in (int,float) and math.isfinite(v) and lo<=v<=hi for v,(lo,hi) in zip(values,LIMITS))
  self.reference=[float(v) for v in values] if valid else self.actual[:]
  status=reply.get('ikStatus','').lower()
  if 'coalesced' in status:
   # A pending Mov already owns its endpoint. Rebase the next live input at
   # that endpoint rather than repeatedly jumping back to stale encoders.
   reference=reply.get('hand_reference',reply['pose'])
   values=[reference.get(k) for k in FIELDS] if isinstance(reference,dict) else []
   self.goal=([float(v) for v in values] if len(values)==6 and all(type(v) in (int,float) and math.isfinite(v) and lo<=v<=hi for v,(lo,hi) in zip(values,LIMITS)) else self.actual[:])
  elif 'blocked' in status:self.goal=self.reference[:] if self.whole_target else self.actual[:]
 def _deltas(self,f,now):
  dt=clamp(now-self.last_time,.001,.1)
  if now-self.last_time>.25:raise UnstableHand('Tracking gap - Space to resume')
  if math.hypot(f['x']-self.last['x'],f['y']-self.last['y'])>.08 or abs(math.log(f['size']/self.last['size']))>.32 or qangle(f['q'],self.last['q'])>math.radians(28):
   raise UnstableHand('Tracking jump - held; Space to resume')
  old=dict(self.filtered);self.last=dict(f);self.last_time=now
  for key,tau in [('x',.035),('y',.035),('size',.075),('grip',.045)]:
   self.filtered[key]+=(f[key]-self.filtered[key])*(1-math.exp(-dt/tau))
  self.filtered['q']=qlerp(self.filtered['q'],f['q'],1-math.exp(-dt/.05))
  delta=qmul(inverse(old['q']),self.filtered['q'])
  if delta[3]<0:delta=[-v for v in delta]
  angle=2*math.acos(clamp(delta[3],-1,1));length=math.sqrt(sum(v*v for v in delta[:3]))
  rotation=[v*angle/max(length,1e-9)*180/math.pi for v in delta[:3]]
  d=[(self.filtered['x']-old['x'])*240,-(self.filtered['y']-old['y'])*180,
     math.log(self.filtered['size']/old['size'])*80,rotation[1]*1.3,rotation[0]*1.3,rotation[2]*1.5]
  thresholds=[10,10,12,14,14,14]
  self.scores=[abs(v)/dt/t for v,t in zip(d,thresholds)]
  # Gain changes angle mapping, not the noise floor or dominant-axis selection.
  d=[v*self.sensitivity for v in d]
  return dt,d
 def update(self,f,now):
  previous_axis=self.active
  dt,d=self._deltas(f,now)
  best=max(range(6),key=lambda i:self.scores[i])
  if self.active is not None:
   if self.scores[self.active]<.45:
    if self.quiet is None:self.quiet=now
    if now-self.quiet>=.10:self.active=None;self.candidate=None;self.quiet=None
   else:self.quiet=None
  if self.active is not None and best!=self.active and self.scores[best]>max(1.5,self.scores[self.active]*2):
   if self.candidate!=best:self.candidate=best;self.candidate_time=now
   elif now-self.candidate_time>=.06:
    self.active=best;self.goal=(self.reference if self.whole_target else self.actual)[:];self.candidate=None;self.quiet=None
  elif self.active is not None:self.candidate=None
  if self.active is None:
   if self.scores[best]>1:
    if best!=self.candidate:self.candidate=best;self.candidate_time=now
    elif now-self.candidate_time>=.045:
     self.active=best;self.goal=(self.reference if self.whole_target else self.actual)[:];self.candidate=None
   else:self.candidate=None
  if self.active is not None and self.scores[self.active]>.45:
   i=self.active;lo,hi=LIMITS[i]
   if previous_axis!=i:self.drive_direction=0;self.reverse_direction=0
   direction=1 if d[i]>0 else -1 if d[i]<0 else 0
   change=d[i]
   if direction and self.drive_direction and direction!=self.drive_direction:
    if self.reverse_direction!=direction:self.reverse_direction=direction;self.reverse_since=now
    if now-self.reverse_since<.03:change=0
    else:self.drive_direction=direction;self.reverse_direction=0
   elif direction:self.drive_direction=direction;self.reverse_direction=0
   self.goal[i]=clamp(self.goal[i]+clamp(change,-100*dt,100*dt),lo,hi)
  self.active_label=LABELS[self.active] if self.active is not None else 'HOLD / move one way'
  g=self.filtered['grip'];g=0 if g<.06 else 1 if g>.94 else g
  return self.goal+[g],self.active if self.active is not None else -1


class MultiMapper(Mapper):
 """Independent hand channels with the same filters, hysteresis and jump guard."""
 def calibrate(self,f,pose,now):
  super().calibrate(f,pose,now)
  self.enabled_axes=[False]*6;self.starts=[None]*6;self.quiet_axes=[None]*6
  self.directions=[0]*6;self.reversals=[0]*6;self.reverse_times=[now]*6
 def update(self,f,now):
  dt,d=self._deltas(f,now)
  for i in range(6):
   score=self.scores[i]
   if not self.enabled_axes[i]:
    if score>1:
     if self.starts[i] is None:self.starts[i]=now
     elif now-self.starts[i]>=.045:
      self.enabled_axes[i]=True;self.goal[i]=self.reference[i];self.directions[i]=0
    else:self.starts[i]=None
   if self.enabled_axes[i]:
    if score<.45:
     if self.quiet_axes[i] is None:self.quiet_axes[i]=now
     elif now-self.quiet_axes[i]>=.1:
      self.enabled_axes[i]=False;self.starts[i]=None;self.quiet_axes[i]=None
    else:self.quiet_axes[i]=None
   if self.enabled_axes[i] and score>.45:
    sign=1 if d[i]>0 else -1 if d[i]<0 else 0;change=d[i]
    if sign and self.directions[i] and sign!=self.directions[i]:
     if self.reversals[i]!=sign:self.reversals[i]=sign;self.reverse_times[i]=now
     if now-self.reverse_times[i]<.03:change=0
     else:self.directions[i]=sign;self.reversals[i]=0
    elif sign:self.directions[i]=sign;self.reversals[i]=0
    lo,hi=LIMITS[i];self.goal[i]=clamp(self.goal[i]+clamp(change,-100*dt,100*dt),lo,hi)
  axes=[i for i in range(6) if self.enabled_axes[i]]
  self.active_label='MULTI '+','.join('J'+str(i+1) for i in axes) if axes else 'HOLD TARGET'
  g=self.filtered['grip'];g=0 if g<.06 else 1 if g>.94 else g
  return self.goal+[g],axes

class Link:
 def __init__(self,port=5008,dry=False):
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',0));self.sock.setblocking(False)
  self.target=('127.0.0.1',port);self.session=str(uuid.uuid4());self.seq=0;self.action=0;self.ack_action=-1;self.ack_command=None;self.reply=None
  self.mode='STOP';self.last_send=-100;self.last_ack=-100;self.ack_seq=-1;self.sent={};self.rtts=[];self.dry=dry
 @property
 def connected(self):return self.dry or time.perf_counter()-self.last_ack<.3
 @property
 def calibrated(self):return self.dry or (self.mode=='CALIBRATE' and self.ack_action==self.action and self.ack_command=='CALIBRATE')
 def send(self,command,pose=None,axis=-1):
  if command!=self.mode:self.action+=1
  self.mode=command;self.seq+=1;now=time.perf_counter();self.last_send=now
  self.sent={k:v for k,v in self.sent.items() if now-v<1};self.sent[self.seq]=now
  p=dict(version=4,session=self.session,sequence=self.seq,action=self.action,command=command,axis=axis,sentMs=int(time.time()*1000))
  if pose is not None:p['pose']=pose
  if not self.dry:self.sock.sendto(json.dumps(p,allow_nan=False).encode(),self.target)
 def poll(self):
  while True:
   try:raw,addr=self.sock.recvfrom(4096)
   except (BlockingIOError,ConnectionResetError):break
   try:
    p=json.loads(raw);seq=p['sequence']
    if addr!=self.target or p['version']!=4 or p['session']!=self.session or seq<=self.ack_seq or seq not in self.sent:continue
    self.reply=p;self.ack_seq=seq;self.ack_action=p['action'];self.ack_command=p['command'];self.last_ack=time.perf_counter();self.rtts.append((self.last_ack-self.sent[seq])*1000)
   except (ValueError,KeyError,TypeError):pass
  if self.mode!='TARGET' and time.perf_counter()-self.last_send>.1:self.send(self.mode)
 def close(self):
  for _ in range(3):self.send('STOP');time.sleep(.01)
  self.sock.close()
