import json
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent
c=json.loads((P/'capture.json').read_text());rr=c['records']
rois=[(275,275,315,305),(275,415,315,445),(100,255,140,285)]
targets={'upper_label':(250,193),'left':(183,385),'right':(383,345)}
def pts(r,d,uv):
 k=r['K']; uv=np.array(uv); z=d[uv[:,1],uv[:,0]]*.001
 cam=np.c_[(uv[:,0]-k[2])/k[0]*z,(uv[:,1]-k[5])/k[4]*z,z];T=np.array(r['camera_to_base']);return cam@T[:3,:3].T+T[:3,3],z>0
out={'scope':'read-only geometry; diagnostic coordinates, not execution permission','roi_pixels':rois,'records':[]}
allpts=[]
for r in rr:
 d=np.load(P/f"depth_{r['index']:02d}.npy")
 uv=[]
 for x0,y0,x1,y1 in rois:
  uv += [(x,y) for y in range(y0,y1,3) for x in range(x0,x1,3)]
 q,v=pts(r,d,uv);q=q[v];A=np.c_[q[:,:2],np.ones(len(q))];coef=np.linalg.lstsq(A,q[:,2],rcond=None)[0];allpts.append(q)
 t={}
 for name,(u,v) in targets.items():
  uv=[(x,y) for y in range(v-5,v+6) for x in range(u-5,u+6)];q1,valid=pts(r,d,uv)
  t[name]={'pixel':[u,v],'surface_point_m':np.median(q1[valid],axis=0).tolist(),'valid_fraction':float(valid.mean()),'depth_median_mm':float(np.median(d[v-5:v+6,u-5:u+6]))}
 T=np.array(r['base_to_Link6']);tcp=(T@np.array([0,0,.138,1]))[:3]
 out['records'].append({'index':r['index'],'stamps_inside_arm_bracket':all(r['pre_arm']['end_wall']<=r[n]<=r['post_arm']['start_wall'] for n in ['rgb_stamp','depth_stamp']),'table_coefficients':coef.tolist(),'table_rmse_mm':float(np.sqrt(np.mean((A@coef-q[:,2])**2))*1000),'targets':t,'tcp_m':tcp.tolist()})
q=np.vstack(allpts);A=np.c_[q[:,:2],np.ones(len(q))];coef=np.linalg.lstsq(A,q[:,2],rcond=None)[0]
out['pooled_table']={'coefficients':coef.tolist(),'rmse_mm':float(np.sqrt(np.mean((A@coef-q[:,2])**2))*1000),'xy_min':q[:,:2].min(0).tolist(),'xy_max':q[:,:2].max(0).tolist(),'z_max_m':float(q[:,2].max())}
for name in targets:
 a=np.array([r['targets'][name]['surface_point_m'] for r in out['records']]);out[name]={'surface_median_m':np.median(a,0).tolist(),'surface_span_mm':(np.ptp(a,axis=0)*1000).tolist(),'note':'central 11x11 observed surface; not sphere center; no historical XY correction'}
(P/'geometry.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='records'},indent=2));print('TCP',out['records'][0]['tcp_m'])
