import cv2,json,numpy as np
from scipy.optimize import least_squares
from pathlib import Path
p=Path('/home/li/桌面/9月机械臂实验（算法）/reports/birdview_return_20260916/after_return');c=json.loads((p/'capture.json').read_text());rows=[]
for r in c['records']:
 if r['sync_error_s']>.001:continue
 i=r['index'];im=cv2.imread(str(p/f'rgb_{i:02d}.jpg'));d=np.load(p/f'depth_{i:02d}.npy');hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);m=cv2.inRange(hsv,(3,100,65),(30,255,255));n,lab,stats,centers=cv2.connectedComponentsWithStats(m);idx=[j for j in range(1,n) if stats[j,4]>2000 and centers[j,0]>340 and centers[j,1]>230];assert len(idx)==1
 mask=np.uint8(lab==idx[0])*255;mask=cv2.erode(mask,np.ones((9,9),np.uint8));y,x=np.where((mask>0)&(d>0));z=d[y,x]*.001;k=r['K'];q=np.c_[(x-k[2])*z/k[0],(y-k[5])*z/k[4],z];q=q[(z>.44)&(z<.56)];A=np.c_[2*q,np.ones(len(q))];sol=np.linalg.lstsq(A,np.sum(q*q,axis=1),rcond=None)[0];r0=np.sqrt(max(1e-6,sol[3]+sum(sol[:3]**2)));fit=least_squares(lambda a:np.linalg.norm(q-a[:3],axis=1)-a[3],np.r_[sol[:3],r0],loss='soft_l1',f_scale=.002,max_nfev=500);T=np.array(r['camera_to_base']);center=(T@np.r_[fit.x[:3],1])[:3];res=np.linalg.norm(q-fit.x[:3],axis=1)-fit.x[3];rows.append(dict(index=i,points=len(q),center_m=center.tolist(),radius_mm=float(fit.x[3]*1000),rmse_mm=float(np.sqrt(np.mean(res**2))*1000),median_abs_residual_mm=float(np.median(np.abs(res))*1000),fit_success=bool(fit.success)))
out={'scope':'Independent exploratory sphere approximation from eroded target mask; no execution or calibration validation','frames':rows};Path(__file__).with_suffix('.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
