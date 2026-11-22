"""Pure hand mapping and UDP transport, independently testable without MediaPipe."""
import math
import json
import socket
import time
import uuid
import threading

FIELDS = ('baseAngle', 'shoulderAngle', 'elbowAngle', 'wristAngle', 'toolAngle', 'forearmAngle', 'grip')
LIMITS = ((-170,170),(-90,135),(-20,171),(-120,120),(-360,360),(-160,160),(0,1))
def clamp(x, lo, hi): return max(lo, min(hi, x))
def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def angle(a,b,c):
    u=[x-y for x,y in zip(a,b)]; v=[x-y for x,y in zip(c,b)]
    n=distance(a,b)*distance(c,b)
    return math.degrees(math.acos(clamp(sum(x*y for x,y in zip(u,v))/max(n,1e-9),-1,1)))

def features(points, width=640, height=480):
    if len(points)!=21 or not all(math.isfinite(v) for p in points for v in p): return None
    if any(not 0.01 < points[i][0] < 0.99 or not 0.01 < points[i][1] < 0.99 for i in (0,4,5,8,9,17)): return None
    p=[(v[0]*width,v[1]*height,v[2]*width) for v in points]
    scale=distance(p[5],p[17])
    if scale < 25: return None
    extended=[angle(p[b],p[b+1],p[b+3]) > 150 for b in (5,9,13,17)]
    folded=[angle(p[b],p[b+1],p[b+3]) < 115 for b in (5,9,13,17)]
    mode_sign=all(folded) and p[4][1]<p[3][1]-scale*.3 and angle(p[1],p[2],p[4])>150
    gesture='MODE' if mode_sign else ('STOP' if all(folded) else ('HOME' if extended[0] and extended[1] and folded[2] and folded[3] else 'MOVE'))
    center=[sum(points[i][axis] for i in (0,5,9,13,17))/5 for axis in (0,1)]
    tilt=math.degrees(math.atan2(p[9][0]-p[0][0],p[0][1]-p[9][1]))
    pinch=clamp((distance(p[4],p[8])/scale-0.15)/0.75,0,1)
    # Palm normal estimates tilt/pronation from the inferred hand depth, not physical depth measurement.
    a=[p[5][i]-p[0][i] for i in range(3)]; b=[p[17][i]-p[0][i] for i in range(3)]
    n=[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    if n[2]<0: n=[-v for v in n]
    yaw=math.degrees(math.atan2(n[0],max(n[2],1e-6)))
    pitch=math.degrees(math.atan2(n[1],math.hypot(n[0],n[2])))
    return (center[0],center[1],tilt,pinch,gesture,scale/width,yaw,pitch)

class Mapper:
    def __init__(self): self.origin=None; self.last=None; self.filtered=None; self.mode='ARM'
    def calibrate(self, f, pose, now):
        self.anchor=[float(pose[k]) for k in FIELDS]
        self.origin=[f[0],f[1],f[2],f[5] if len(f)>5 else .15,f[6] if len(f)>6 else 0,f[7] if len(f)>7 else 0]
        self.filtered=list(self.origin)+[f[3]]; self.last=now; self.output=list(self.anchor)
    def update(self,f,now):
        dt=clamp(now-self.last,.001,.1); self.last=now
        values=[f[0],f[1],f[2],f[5] if len(f)>5 else .15,f[6] if len(f)>6 else 0,f[7] if len(f)>7 else 0,f[3]]
        for i,v in enumerate(values):
            if i in (2,4,5): v=self.filtered[i]+((v-self.filtered[i]+180)%360-180)
            speed=abs(v-self.filtered[i])/dt
            tau=.018 if speed>(90 if i in (2,4,5) else .8) else .045
            self.filtered[i]+=(v-self.filtered[i])*(1-math.exp(-dt/tau))
        x,y,roll,size,yaw,pitch,grip=self.filtered
        def dead(v,d): return math.copysign(max(0,abs(v)-d),v)
        p=list(self.anchor)
        if self.mode=='ARM':
            p[0]+=dead(x-self.origin[0],.008)*240
            p[1]+=dead(y-self.origin[1],.008)*100
            # Apparent palm size supplies a relative toward/away input; no metric Z claim.
            p[2]+=dead(math.log(max(size,.02)/max(self.origin[3],.02)),.035)*90
        else:
            p[5]+=dead(yaw-self.origin[4],2)*1.5
            p[3]+=dead(pitch-self.origin[5],2)
            p[4]+=dead((roll-self.origin[2]+180)%360-180,2)*2
        p[6]=0 if grip<.04 else (1 if grip>.96 else grip)
        for i,(lo,hi) in enumerate(LIMITS):
            value=clamp(p[i],lo,hi)
            if abs(value-self.output[i])>(.008 if i==6 else .2) or value in (lo,hi): self.output[i]=value
        return list(self.output)

class Link:
    def __init__(self,port=5006,dry=False):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.bind(('127.0.0.1',0)); self.sock.setblocking(False)
        self.target=('127.0.0.1',port); self.session=str(uuid.uuid4()); self.seq=0; self.action=0
        self.mode='STOP'; self.last_send=-100; self.last_ack=-100; self.ack_seq=-1; self.pose=None; self.sent={}; self.rtts=[]; self.dry=dry
        if dry: self.pose=dict(zip(FIELDS,[0,-15,50,55,0,0,1]))
    @property
    def connected(self): return self.dry or time.perf_counter()-self.last_ack<0.3
    def send(self,command,pose=None):
        if command!=self.mode: self.action+=1
        self.mode=command; self.seq+=1
        now=time.perf_counter(); self.last_send=now; self.sent[self.seq]=now
        self.sent={k:v for k,v in self.sent.items() if now-v<1}
        p=dict(version=2,session=self.session,sequence=self.seq,action=self.action,command=command,sentMs=int(time.time()*1000))
        if pose is not None: p['pose']=pose
        if not self.dry: self.sock.sendto(json.dumps(p,allow_nan=False).encode(),self.target)
    def poll(self):
        while True:
            try: raw,addr=self.sock.recvfrom(4096)
            except (BlockingIOError,ConnectionResetError): break
            try:
                p=json.loads(raw); seq=p.get('sequence',-1)
                if addr!=self.target or p.get('version')!=2 or p.get('session')!=self.session or seq<=self.ack_seq or seq not in self.sent: continue
                self.ack_seq=seq; self.last_ack=time.perf_counter(); self.pose=p['pose']; self.rtts.append((self.last_ack-self.sent[seq])*1000)
            except (ValueError,KeyError,TypeError): pass
        # Never repeat a stale continuous pose. Fresh results alone sustain movement.
        if self.mode!='POSE' and time.perf_counter()-self.last_send>0.1: self.send(self.mode)
    def close(self):
        for _ in range(3): self.send('STOP'); time.sleep(0.01)
        self.sock.close()


class LatestCamera:
    def __init__(self,cv2,index,width,height):
        self.cap=cv2.VideoCapture(index)
        if not self.cap.isOpened(): self.cap.release(); raise RuntimeError('Cannot open camera')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,width); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        self.cap.set(cv2.CAP_PROP_FPS,60); self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.lock=threading.Lock(); self.frame=None; self.stamp=0; self.failed=False; self.stop=threading.Event()
        self.thread=threading.Thread(target=self.read,daemon=True); self.thread.start()
    def read(self):
        while not self.stop.is_set():
            ok,frame=self.cap.read()
            with self.lock:
                if not ok: self.failed=True; return
                self.frame=frame; self.stamp=time.perf_counter()
    def latest(self):
        with self.lock: return self.frame,self.stamp,self.failed
    def close(self):
        self.stop.set(); self.thread.join(timeout=1); self.cap.release(); self.thread.join(timeout=1)
