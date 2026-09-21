from pathlib import Path
import json,numpy as np
src=Path(__file__).resolve().parent.parent/'new_table_20260915'
c=json.loads((src/'capture.json').read_text());rois=[[30,350,150,450],[350,360,500,450],[400,235,450,290]];rows=[];allp=[]
for r in c['records']:
 assert r['pre_arm']['end_wall']<min(r['rgb_stamp'],r['depth_stamp'])<=max(r['rgb_stamp'],r['depth_stamp'])<r['post_arm']['start_wall']
 d=np.load(src/f"depth_{r['index']:02d}.npy");K=np.array(r['K']).reshape(3,3);T=np.array(r['camera_to_base']);pts=[]
 for x0,y0,x1,y1 in rois:
  v,u=np.mgrid[y0:y1:3,x0:x1:3];z=d[v,u].ravel()*.001;u=u.ravel();v=v.ravel();valid=(z>.15)&(z<1.5);z=z[valid];u=u[valid];v=v[valid]
  p=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z];pts.append(p@T[:3,:3].T+T[:3,3])
 p=np.concatenate(pts);A=np.c_[p[:,:2],np.ones(len(p))];mask=np.ones(len(p),bool)
 for _ in range(5):
  coef=np.linalg.lstsq(A[mask],p[mask,2],rcond=None)[0];res=p[:,2]-A@coef;mask=np.abs(res-np.median(res))<.004
 rows.append({'index':r['index'],'coef_z_ax_by_c':coef.tolist(),'rmse_mm':float(np.sqrt(np.mean(res[mask]**2))*1000),'kept':int(mask.sum()),'points':len(p)});allp.append(p[mask])
P=np.concatenate(allp);A=np.c_[P[:,:2],np.ones(len(P))];coef=np.linalg.lstsq(A,P[:,2],rcond=None)[0]
out={'rois':rois,'all_timestamps_bracketed':True,'D_first':c['records'][0]['D'],'coef_z_ax_by_c':coef.tolist(),'pooled_rmse_mm':float(np.sqrt(np.mean((P[:,2]-A@coef)**2))*1000),'xy_min':P[:,:2].min(0).tolist(),'xy_max':P[:,:2].max(0).tolist(),'frame_coefficients':rows,'method':'Independent ROIs, 3-pixel subsampling, iterated 4mm vertical residual rejection; no absolute calibration claim'}
Path(__file__).with_name('independent_fit.json').write_text(json.dumps(out,indent=2));print({k:v for k,v in out.items() if k!='frame_coefficients'})
