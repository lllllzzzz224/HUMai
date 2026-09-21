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
out=[];label=im.copy();plane=np.array(json.loads((P/'table_fit.json').read_text())['plane_z_equals_ax_by_c'])
for i,box in enumerate(result.boxes):
 if result.names[int(box.cls)]!='orange':continue
 xy=box.xyxy[0].cpu().numpy();mask=AppleCenterLocalizer.detection_mask(result,i,xy,im.shape[:2]);ys,xs=np.where(mask);u=float(np.median(xs));v=float(np.median(ys));idx=len(out);rows=[]
 cv2.imwrite(str(P/f'orange_mask_{idx}.png'),mask.astype('uint8')*255)
 for rec in c['records']:
  d=np.load(P/f"depth_{rec['index']:02d}.npy");T=np.array(rec['camera_to_base']);state={'method':'central_depth_radius_fallback'}
  def sphere(*args,**kwargs):
   fit=AppleCenterLocalizer.ransac_sphere_fit(*args,**kwargs)
   if fit is not None:state.update(method='legacy_ransac_sphere',sphere_rmse_m=float(fit[2]))
   return fit
  stub=types.SimpleNamespace(camera_info=types.SimpleNamespace(k=rec['K']),stats={},ransac_sphere_fit=sphere,get_logger=lambda:Log())
  np.random.seed(15000+idx*100+rec['index']);geom=AppleCenterLocalizer.estimate_points(stub,mask,xy,d)
  raw=d[mask];row={'frame':rec['index'],'depth_valid_fraction':float(np.mean((raw>200)&(raw<1000)))}
  if geom is not None:
   surf,center,r,n=geom;sb=T[:3,:3]@surf+T[:3,3];cb=T[:3,:3]@center+T[:3,3];row.update(state,surface_camera_m=surf.tolist(),estimated_center_camera_m=center.tolist(),surface_base_m=sb.tolist(),estimated_center_base_m=cb.tolist(),estimated_radius_m=r,valid_points=n,estimated_center_above_candidate_plane_m=float(cb[2]-np.r_[cb[:2],1]@plane))
  else:row.update(method='rejected',reason=stub.stats)
  rows.append(row)
 edge=bool(xy[0]<=1 or xy[1]<=1 or xy[2]>=im.shape[1]-1 or xy[3]>=im.shape[0]-1)
 location='basin_inside_image_interpretation' if u<100 and v<215 else 'on_paper_cluster'
 centers=np.array([r['estimated_center_base_m'] for r in rows if 'estimated_center_base_m'in r]);surfaces=np.array([r['surface_base_m'] for r in rows if 'surface_base_m'in r])
 item={'id':idx,'confidence':float(box.conf),'bbox_xyxy':xy.tolist(),'mask_median_uv':[u,v],'mask_pixels':int(mask.sum()),'mask_type':'segmentation' if result.masks is not None else 'bbox_fallback','edge_truncated':edge,'context':location,'tracking':'first-frame mask applied to all static-camera frames; no target ID claim across motion','samples':rows,'usable_3d':False,'three_d_status':'diagnostic_only: paper plane and absolute geometry not validated'}
 if len(centers):item.update(center_span_m=np.ptp(centers,axis=0).tolist(),surface_span_m=np.ptp(surfaces,axis=0).tolist(),diagnostic_median_surface_base_m=np.median(surfaces,axis=0).tolist())
 out.append(item);x0,y0,x1,y1=xy.astype(int);cv2.rectangle(label,(x0,y0),(x1,y1),(0,200,255),2);cv2.putText(label,f'O{idx} {float(box.conf):.2f}',(x0,max(15,y0)),0,.5,(0,0,255),2)
cv2.imwrite(str(P/'orange_detection.jpg'),label);(P/'orange_localization_diagnostic.json').write_text(json.dumps({'model':str(default_model_path()),'no_ros_publish':True,'historical_xy_correction_applied':False,'detections':out},indent=2));print(json.dumps([{'id':o['id'],'uv':o['mask_median_uv'],'confidence':o['confidence'],'edge':o['edge_truncated'],'context':o['context'],'center_span_m':o.get('center_span_m')}for o in out]))
