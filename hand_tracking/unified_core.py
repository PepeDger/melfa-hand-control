"""Unified Cartesian hand input. No joint selection and no real-robot transport."""
import math
import json
import socket
import time
import uuid
import sys
from pathlib import Path
from .hand_core import LatestCamera, features as old_features

def norm(v):return math.sqrt(sum(x*x for x in v))
def clamp(v,a,b):return max(a,min(b,v))
def unit(v):
 n=norm(v)
 if n<1e-8:raise ValueError('Degenerate hand basis')
 return [x/n for x in v]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def qmul(a,b):
 x,y,z,w=a;X,Y,Z,W=b
 return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def inverse(q):return [-q[0],-q[1],-q[2],q[3]]
def qangle(a,b):return 2*math.acos(clamp(abs(sum(x*y for x,y in zip(a,b))),0,1))
def qlerp(a,b,t):
 if sum(x*y for x,y in zip(a,b))<0:b=[-v for v in b]
 return unit([x+(y-x)*t for x,y in zip(a,b)])
def basis_quaternion(right,up,normal):
 # Matrix columns are an orthonormal palm frame; avoid Euler wrap/normal sign flips.
 m=[[right[i],up[i],normal[i]] for i in range(3)];trace=sum(m[i][i] for i in range(3))
 if trace>0:
  s=math.sqrt(trace+1)*2;q=[(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s,s/4]
 else:
  i=max(range(3),key=lambda i:m[i][i]);j=(i+1)%3;k=(i+2)%3;s=math.sqrt(1+m[i][i]-m[j][j]-m[k][k])*2
  q=[0,0,0,0];q[i]=s/4;q[j]=(m[j][i]+m[i][j])/s;q[k]=(m[k][i]+m[i][k])/s;q[3]=(m[k][j]-m[j][k])/s
 return unit(q)
def sample(points,width=640,height=480):
 f=old_features(points,width,height)
 if f is None:return None
 p=[[v[0]*width,-v[1]*height,-v[2]*width] for v in points]
 try:
  right=unit([p[5][i]-p[17][i] for i in range(3)])
  up0=[p[9][i]-p[0][i] for i in range(3)]
  normal=unit(cross(right,up0));up=unit(cross(normal,right))
  # Edge-on palms and collapsed depth geometry are too ambiguous for teleoperation.
  if abs(normal[2])<.22:return None
  gesture='STOP' if f[4]=='MODE' else f[4]
  return dict(x=f[0],y=f[1],size=f[5],q=basis_quaternion(right,up,normal),grip=f[3],gesture=gesture)
 except ValueError:return None

class UnstableHand(ValueError):pass
class Mapper:
 def calibrate(self,f,now):
  self.origin=dict(f);self.last=dict(f);self.filtered=dict(f);self.last_time=now
  self.offset=[0,0,0];self.rotation=[0,0,0,1];self.grip=f['grip']
 def update(self,f,now):
  dt=clamp(now-self.last_time,.001,.1)
  if now-self.last_time>.25:raise UnstableHand('Tracking gap - recalibrate')
  # Reject discontinuities before smoothing; never turn a bad depth sample into a robot target.
  if (math.hypot(f['x']-self.last['x'],f['y']-self.last['y'])>.08
      or abs(math.log(f['size']/self.last['size']))>.32
      or qangle(f['q'],self.last['q'])>math.radians(28)):
   raise UnstableHand('Hand estimate jumped - held; Space to resume')
  self.last=dict(f);self.last_time=now
  for key,tau in [('x',.10),('y',.10),('size',.20),('grip',.08)]:
   self.filtered[key]+=(f[key]-self.filtered[key])*(1-math.exp(-dt/tau))
  self.filtered['q']=qlerp(self.filtered['q'],f['q'],1-math.exp(-dt/.14))
  def dead(v,d):return math.copysign(max(0,abs(v)-d),v)
  desired=[clamp(dead(self.filtered['x']-self.origin['x'],.008)*1.4,-.4,.4),
           clamp(dead(self.origin['y']-self.filtered['y'],.008)*1.1,-.3,.3),
           clamp(dead(math.log(self.filtered['size']/self.origin['size']),.04)*.45,-.25,.25)]
  delta=[v-x for v,x in zip(desired,self.offset)];d=norm(delta);step=.25*dt
  if d>step:delta=[v*step/d for v in delta]
  self.offset=[v+x for v,x in zip(delta,self.offset)]
  q=qmul(inverse(self.origin['q']),self.filtered['q']);a=qangle(q,[0,0,0,1])
  if a>math.radians(40):q=qlerp([0,0,0,1],q,math.radians(40)/a)
  a=qangle(self.rotation,q);self.rotation=qlerp(self.rotation,q,min(1,math.radians(50)*dt/max(a,1e-8)))
  grip=self.filtered['grip'];grip=0 if grip<.06 else 1 if grip>.94 else grip
  self.grip+=clamp(grip-self.grip,-1.5*dt,1.5*dt)
  return self.offset+self.rotation+[self.grip]

class Link:
 def __init__(self,port=5007,dry=False):
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',0));self.sock.setblocking(False)
  self.target=('127.0.0.1',port);self.session=str(uuid.uuid4());self.seq=0;self.action=0;self.ack_action=-1;self.ack_command=None;self.reply=None
  self.mode='STOP';self.last_send=-100;self.last_ack=-100;self.ack_seq=-1;self.sent={};self.rtts=[];self.dry=dry
 @property
 def connected(self):return self.dry or time.perf_counter()-self.last_ack<.3
 @property
 def calibrated(self):return self.dry or (self.mode=='CALIBRATE' and self.ack_action==self.action and self.ack_command=='CALIBRATE')
 def send(self,command,pose=None):
  if command!=self.mode:self.action+=1
  self.mode=command;self.seq+=1;now=time.perf_counter();self.last_send=now
  self.sent={k:v for k,v in self.sent.items() if now-v<1};self.sent[self.seq]=now
  p=dict(version=3,session=self.session,sequence=self.seq,action=self.action,command=command,sentMs=int(time.time()*1000))
  if pose is not None:p['pose']=pose
  if not self.dry:self.sock.sendto(json.dumps(p,allow_nan=False).encode(),self.target)
 def poll(self):
  while True:
   try:raw,addr=self.sock.recvfrom(4096)
   except (BlockingIOError,ConnectionResetError):break
   try:
    p=json.loads(raw);seq=p['sequence']
    if addr!=self.target or p['version']!=3 or p['session']!=self.session or seq<=self.ack_seq or seq not in self.sent:continue
    self.reply=p;self.ack_seq=seq;self.ack_action=p['action'];self.ack_command=p['command'];self.last_ack=time.perf_counter();self.rtts.append((self.last_ack-self.sent[seq])*1000)
   except (ValueError,KeyError,TypeError):pass
  if self.mode!='TARGET' and time.perf_counter()-self.last_send>.1:self.send(self.mode)
 def close(self):
  for _ in range(3):self.send('STOP');time.sleep(.01)
  self.sock.close()
