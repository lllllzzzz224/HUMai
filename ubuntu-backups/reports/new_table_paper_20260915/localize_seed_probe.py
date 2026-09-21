import sys,json,types
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/li/hand_eye_calibration/apple_pick_v2')
from ultralytics import YOLO
from project_paths import default_model_path
from apple_center_localizer_v2 import AppleCenterLocalizer
P=Path(__file__).parent;c=json.loads((P/'capture.json').read_text());im=cv2.imread(str(P/'rgb_00.jpg'));model=YOLO(str(default_model_path()));result=model.predict(im,conf=.15,verbose=False,device='cpu')[0]
class Log:
 def info(self,*a,**k):pass
 def warning(self,*a,**k):pass
out=[]
for i,box in enumerate(result.boxes):
 if result.names[int(box.cls)]!='orange':continue
 xy=box.xyxy[0].cpu().numpy();mask=AppleCenterLocalizer.detection_mask(result,i,xy,im.shape[:2]);ys,xs=np.where(mask)
 if abs(np.median(xs)-249)>5 or abs(np.median(ys)-193)>5:continue
 for mode in ['fixed_seed_across_frames','multiple_seeds_same_frame']:
  vals=[]
  for j in range(25):
   rec=c['records'][j if mode=='fixed_seed_across_frames' else 0];d=np.load(P/f"depth_{rec['index']:02d}.npy");T=np.array(rec['camera_to_base']);state={'method':'fallback'}
   def sphere(*a,**kw):
    f=AppleCenterLocalizer.ransac_sphere_fit(*a,**kw)
    if f is not None:state['method']='sphere'
    return f
   stub=types.SimpleNamespace(camera_info=types.SimpleNamespace(k=rec['K']),stats={},ransac_sphere_fit=sphere,get_logger=lambda:Log())
   seed=123 if mode=='fixed_seed_across_frames' else j
   np.random.seed(seed);g=AppleCenterLocalizer.estimate_points(stub,mask,xy,d)
   if g is not None:vals.append(dict(frame=rec['index'],seed=seed,method=state['method'],center_base_m=(T[:3,:3]@g[1]+T[:3,3]).tolist()))
  arr=np.array([x['center_base_m']for x in vals]);out.append(dict(mode=mode,center_span_m=np.ptp(arr,axis=0).tolist(),samples=vals))
(P/'ransac_randomness_probe.json').write_text(json.dumps(out,indent=2));print(json.dumps([{k:v for k,v in x.items()if k!='samples'}for x in out]))
