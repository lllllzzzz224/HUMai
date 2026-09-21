from pathlib import Path
import json,cv2,numpy as np
from scipy.spatial import ConvexHull
P=Path(__file__).parent;c=json.loads((P/'capture.json').read_text())
# Visible bare green tabletop only. Avoid basin, cable/seam, fruit and floor.
rois=[[345,350,455,440],[455,390,535,455]]
stack=np.stack([np.load(P/f"depth_{r['index']:02d}.npy") for r in c["records"]]).astype(float)
stack[stack==0]=np.nan
median=np.nanmedian(stack,axis=0)
spread=np.nanquantile(stack,.9,axis=0)-np.nanquantile(stack,.1,axis=0)
stable=(np.sum(np.isfinite(stack),axis=0)>=20)&(spread<=10)
np.save(P/"temporal_stable_mask.npy",stable)
allpts=[];stats=[]
for rec in c['records']:
 d=np.load(P/f"depth_{rec['index']:02d}.npy");K=np.array(rec['K']).reshape(3,3);T=np.array(rec['camera_to_base'])
 for ri,(x0,y0,x1,y1) in enumerate(rois):
  yy,xx=np.mgrid[y0:y1:3,x0:x1:3];z=d[yy,xx].astype(float)*.001
  good=(z>.1)&(z<2)&stable[yy,xx];xyz=np.stack(((xx-K[0,2])*z/K[0,0],(yy-K[1,2])*z/K[1,1],z),axis=-1)[good]
  pts=xyz@T[:3,:3].T+T[:3,3];allpts.append(pts);stats.append({'frame':rec['index'],'roi':ri,'n':len(pts),'median_xyz_m':np.median(pts,axis=0).tolist()})
pts=np.concatenate(allpts);rng=np.random.default_rng(15);best=None
for _ in range(400):
 p=pts[rng.choice(len(pts),3,replace=False)];normal=np.cross(p[1]-p[0],p[2]-p[0]);norm=np.linalg.norm(normal)
 if norm<1e-10:continue
 normal/=norm
 if abs(normal[2])<.8:continue
 keep=np.abs((pts-p[0])@normal)<.005
 if best is None or keep.sum()>best.sum():best=keep
if best is None:raise RuntimeError('No tabletop plane')
def fit(p):
 A=np.c_[p[:,:2],np.ones(len(p))];v=np.linalg.lstsq(A,p[:,2],rcond=None)[0];return v
v=fit(pts[best]);res=pts[:,2]-(np.c_[pts[:,:2],np.ones(len(pts))]@v);keep=np.abs(res)<.005;v=fit(pts[keep]);res=pts[:,2]-np.c_[pts[:,:2],np.ones(len(pts))]@v
hull=ConvexHull(pts[keep,:2]);poly=pts[keep,:2][hull.vertices]
frames=[]
for r in c['records']:
 ps=np.concatenate([allpts[i] for i,s in enumerate(stats) if s['frame']==r['index']]);e=ps[:,2]-np.c_[ps[:,:2],np.ones(len(ps))]@v
 frames.append({'frame':r['index'],'median_residual_m':float(np.median(e)),'p95_abs_residual_m':float(np.quantile(abs(e),.95))})
result={'plane_z_equals_ax_by_c':v.tolist(),'rois_xyxy':rois,'point_count':len(pts),'inlier_fraction':float(np.mean(keep)),'inlier_rmse_m':float(np.sqrt(np.mean(res[keep]**2))),'all_p95_abs_residual_m':float(np.quantile(abs(res),.95)),'sample_support_hull_xy_m':poly.tolist(),'roi_statistics':stats,'frame_consistency':frames,'measured_inlier_z_min_max_m':[float(pts[keep,2].min()),float(pts[keep,2].max())],'temporal_filter':'>=20/25 valid samples, p90-p10 depth <=10mm; left two ROIs rejected for 48/81mm temporal Z drift','warning':'Sample hull is observation coverage, NOT table physical boundary. Plane fit residual is NOT absolute hand-eye accuracy.'}
(P/'table_fit.json').write_text(json.dumps(result,indent=2));np.save(P/'table_points.npy',pts)
im=cv2.imread(str(P/'rgb_00.jpg'))
for i,(x0,y0,x1,y1) in enumerate(rois):cv2.rectangle(im,(x0,y0),(x1,y1),(0,255,255),2);cv2.putText(im,f'ROI {i}',(x0,y0-5),0,.5,(0,255,255),1)
cv2.imwrite(str(P/'table_rois.jpg'),im)
print(json.dumps({k:result[k] for k in ['plane_z_equals_ax_by_c','point_count','inlier_fraction','inlier_rmse_m','all_p95_abs_residual_m','measured_inlier_z_min_max_m']}))
