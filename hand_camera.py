"""Continuous wrist/finger control. Space calibrates/resumes; fist STOP; V sign HOME."""
import argparse
from pathlib import Path
import sys
import threading
import time
import statistics
from hand_tracking.direct_core import sample, Mapper, MultiMapper, LatestCamera, UnstableHand, HOME
from hand_link import Link
from hand_settings import validate_sensitivity
ROOT=Path(__file__).resolve().parent




def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera',type=int,default=0)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--model',type=Path,default=ROOT/'hand_landmarker.task')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--web-ui',action='store_true',help='Preview and input in web panel, no native camera window')
    parser.add_argument('--check-model',action='store_true')
    parser.add_argument('--frames',type=int,default=0,help='Exit after N results')
    parser.add_argument('--sensitivity',type=float,default=1.0,help='Movement gain, 0.5..50.0')
    args=parser.parse_args()
    try: args.sensitivity=validate_sensitivity(args.sensitivity)
    except ValueError as exc: parser.error(str(exc))
    import cv2
    import numpy as np
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    lock=threading.Lock(); mailbox=[None]; ready=threading.Event()
    def callback(result,image,stamp):
        with lock: mailbox[0]=(result,stamp,time.perf_counter())
        ready.set()
    options=vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_buffer=args.model.read_bytes()),
        running_mode=vision.RunningMode.LIVE_STREAM,num_hands=2,
        min_hand_detection_confidence=0.6,min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,result_callback=callback)
    camera=None; link=None; ages=[]; active=False; pending=False; mapper=Mapper(args.sensitivity)
    status='SPACE: calibrate and enable'; label=None; home_start=None
    last_input=last_result=-1; frame_count=0; last_valid=time.perf_counter(); start=last_valid
    points=[]; last_draw=0;last_preview=0
    try:
        with vision.HandLandmarker.create_from_options(options) as detector:
            if args.check_model:
                t=time.perf_counter(); detector.detect_async(mp.Image(image_format=mp.ImageFormat.SRGB,data=np.zeros((480,640,3),dtype=np.uint8)),int(t*1000))
                if not ready.wait(5): raise RuntimeError('No async model result')
                print('MODEL OK / async callback: {:.1f} ms'.format((time.perf_counter()-t)*1000)); return
            link=Link(args.port,args.dry_run)
            camera=LatestCamera(cv2,args.camera,640,480)
            print('Space: enable/calibrate | C: recalibrate | fist: STOP | V sign: HOME | Q: quit',flush=True)
            while True:
                link.poll(); now=time.perf_counter()
                if link.reply and 'sensitivity' in link.reply:
                    mapper.sensitivity=validate_sensitivity(link.reply['sensitivity'])
                if link.reply and link.reply.get('command')=='EXIT': break
                if active and link.reply and link.reply.get('command')=='STOP':
                    active=pending=False
                    status=link.reply.get('ikStatus','STOP - SPACE to resume')
                    link.send('STOP')
                frame,captured,failed=camera.latest()
                if failed: raise RuntimeError('Camera disconnected')
                if frame is None:
                    if now-start>5: raise RuntimeError('Camera timeout')
                    time.sleep(0.005); continue
                # Only submit genuinely fresh frames; MediaPipe drops input when busy.
                stamp=int(captured*1000)
                display=cv2.flip(frame,1)
                if stamp>last_input and now-captured<0.2:
                    detector.detect_async(mp.Image(image_format=mp.ImageFormat.SRGB,data=cv2.cvtColor(display,cv2.COLOR_BGR2RGB)),stamp)
                    last_input=stamp
                with lock: item=mailbox[0]; mailbox[0]=None
                if item is not None:
                    result,t,completed=item
                    if t>last_result:
                        last_result=t; frame_count+=1
                        age=now-t/1000; ages.append((completed-t/1000)*1000)
                        points=[]; f=None; detected=None
                        if len(result.hand_landmarks)==1 and age<0.25:
                            points=[(p.x,p.y,p.z) for p in result.hand_landmarks[0]]
                            f=sample(points,frame.shape[1],frame.shape[0])
                            detected=result.handedness[0][0].category_name
                        if f is None or (active and detected!=label):
                            if active or pending or link.mode!='STOP': link.send('STOP')
                            active=pending=False; home_start=None
                            status='Hand lost/ambiguous/stale - SPACE to resume'
                        else:
                            last_valid=now
                            if f['gesture']=='STOP':
                                link.send('STOP'); active=pending=False; home_start=None; status='FIST / STOP'
                            elif f['gesture']=='HOME' and active:
                                link.send('HOME'); active=pending=False
                                status='HOME requested - confirm in web panel'; home_start=None
                            else:
                                if home_start is not None and active:
                                    link.send('CALIBRATE'); active=False; pending=True
                                home_start=None
                                if pending and link.connected and link.mode=='STOP': link.send('CALIBRATE')
                                if pending and f['gesture']=='MOVE' and link.connected and link.calibrated:
                                    mapper=(MultiMapper if link.reply and link.reply.get('multi_axis') else Mapper)(args.sensitivity)
                                    if link.reply and 'sensitivity' in link.reply:mapper.sensitivity=validate_sensitivity(link.reply['sensitivity'])
                                    mapper.calibrate(f,(link.reply['pose'] if link.reply else HOME),now); active=True; pending=False; label=detected
                                if active and f['gesture']=='MOVE':
                                    try:
                                        mapper.reconcile(link.reply)
                                        target,axis=mapper.update(f,now)
                                        link.send('TARGET',target,-1 if isinstance(axis,list) else axis,age_ms=max(0,(time.perf_counter()-t/1000)*1000),axes=axis if isinstance(axis,list) else None); status=mapper.active_label
                                    except UnstableHand as error:
                                        link.send('STOP'); active=pending=False; status=str(error)
                if (active or pending) and (now-last_valid>0.25 or not link.connected):
                    link.send('STOP'); active=pending=False; status='Tracking/link timeout - SPACE to resume'
                if not link.connected:
                    if link.mode!='STOP': link.send('STOP')
                    status='Waiting for web hand mode / enable in panel'
                # GUI is intentionally separate from result handling and network sending.
                if now-last_draw>1/30:
                    last_draw=now
                    for p in points:
                        cv2.circle(display,(int(p[0]*display.shape[1]),int(p[1]*display.shape[0])),3,(40,230,150),-1)
                    lines=[status,'SPACE: enable/pause  C: calibrate  Q: exit',
                           'J1 L/R | J2 Up/Down | J3 Near/Far',
                           'J4 palm turn | J5 palm tilt | J6 wrist roll',
                           'Pinch grip | Fist STOP | V HOME | C re-center',
                           ('WEB: '+link.reply.get('ikStatus','')) if link.reply else 'WEB: waiting',
                           'Capture->result: {:.0f} ms | HTTP RTT: {:.0f} ms'.format(ages[-1] if ages else 0,link.rtts[-1] if link.rtts else 0)]
                    for i,s in enumerate([] if args.web_ui else lines):
                        cv2.putText(display,s,(10,22+24*i),cv2.FONT_HERSHEY_SIMPLEX,0.48,(0,0,0),3)
                        cv2.putText(display,s,(10,22+24*i),cv2.FONT_HERSHEY_SIMPLEX,0.48,(80,255,170),1)
                    if args.web_ui:
                        if now-last_preview>.1:
                            ok,jpeg=cv2.imencode('.jpg',display,[cv2.IMWRITE_JPEG_QUALITY,60])
                            if ok: link.preview(jpeg.tobytes())
                            last_preview=now
                    else: cv2.imshow('Web hand control - MELFA',display)
                key=link.take_key() if args.web_ui else cv2.waitKey(1)&255
                if key in (ord('q'),27) or (not args.web_ui and cv2.getWindowProperty('Web hand control - MELFA',cv2.WND_PROP_VISIBLE)<1): break
                if key in (ord(' '),ord('c')):
                    resume=(not active) or key==ord('c')
                    link.send('STOP'); active=False; pending=resume; last_valid=now
                    if resume and link.connected: link.send('CALIBRATE')
                    status='Show one hand to calibrate' if pending else 'PAUSED'
                if args.frames and frame_count>=args.frames: break
                if now-captured>3: raise RuntimeError('Camera stalled')
                time.sleep(0.002)
    finally:
        if link: link.close()
        if camera: camera.close()
        if not args.web_ui: cv2.destroyAllWindows()
        for name,values in [('capture_to_result_ms',ages),('http_roundtrip_ms',link.rtts if link else [])]:
            if values:
                ordered=sorted(values)
                print('{}: n={} median={:.1f} p95={:.1f}'.format(name,len(values),statistics.median(values),ordered[int((len(ordered)-1)*0.95)]))

if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: pass
    except (OSError,RuntimeError,ValueError) as e: print('ERROR:',e); raise SystemExit(1)

