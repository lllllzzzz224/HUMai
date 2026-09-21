import json,cv2,numpy as np
from pathlib import Path
P=Path(__file__).resolve().parent;c=json.loads((P/'capture.json').read_text());out={'scope':'full color contour plus visible surface; not fruit center; old hand-eye conditional','frames':[],'capture_errors':c['errors'],'excluded_nonsynchronous_indices':[r['index'] for r in c['records'] if r['sync_error_s']>1e-6]};contour3ds=[]
for r in c['records']:
 if r['sync_error_s']>1e-6:continue
 im=cv2.imread(str(P/f"rgb_{r['index']:02d}.jpg"));d=np.load(P/f"depth_{r['index']:02d}.npy");hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,(3,90,60),(30,255,255));cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cs=[x for x in cs if cv2.contourArea(x)>2000];cs=sorted(cs,key=lambda x:cv2.boundingRect(x)[0]);assert len(cs)==3
 rr={'index':r['index'],'objects':[]};k=r['K'];T=np.array(r['camera_to_base'])
 for j,cont in enumerate(cs):
  x,y,w,h=cv2.boundingRect(cont);m=cv2.moments(cont);u=m['m10']/m['m00'];v=m['m01']/m['m00'];u0,v0=round(u),round(v);patch=d[v0-5:v0+6,u0-5:u0+6];z=np.median(patch[patch>0])*.001;point=T@np.array([(u-k[2])/k[0]*z,(v-k[5])/k[4]*z,z,1]);obj={'id':['left','label','right_target'][j],'bbox_xywh':[x,y,w,h],'contour_centroid_pixel':[u,v],'touches_image_boundary':bool(x<=0 or y<=0 or x+w>=640 or y+h>=480),'patch_depth_mm':float(z*1000),'patch_valid_fraction':float(np.mean(patch>0)),'visible_surface_m':point[:3].tolist()};rr['objects'].append(obj)
  full=np.zeros(d.shape,np.uint8);cv2.drawContours(full,[cont],-1,255,-1);obj['full_mask_depth_valid_fraction']=float(np.mean(d[full>0]>0));obj['mask_pixels']=int(np.sum(full>0));cv2.drawContours(im,[cont],-1,(0,255,255),2);cv2.putText(im,obj['id'],(x,y-8),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1);cv2.circle(im,(u0,v0),4,(0,0,255),2)
  if j==2 and r['index']==0:
   yy,xx=np.where((full>0)&(d>0));zz=d[yy,xx]*.001;cam=np.c_[(xx-k[2])/k[0]*zz,(yy-k[5])/k[4]*zz,zz];contour3ds=cam@T[:3,:3].T+T[:3,3]
 rr['timestamps_bracketed']=all(r['pre_arm']['end_wall']<=r[n]<=r['post_arm']['start_wall'] for n in ['rgb_stamp','depth_stamp']);out['frames'].append(rr)
 if r['index']==0:cv2.imwrite(str(P/'annotated_contours.jpg'),im)
a=np.array([next(x for x in r['objects'] if x['id']=='right_target')['visible_surface_m'] for r in out['frames']]);out['right_target_surface_median_m']=np.median(a,axis=0).tolist();out['right_target_surface_span_mm']=(np.ptp(a,axis=0)*1000).tolist()
r=c['records'][0];L=np.array(r['base_to_Link6']);tcp=(L@np.array([0,0,.138,1]))[:3];T=np.array(r['camera_to_base']);k=np.array(r['K']).reshape(3,3);out['current_tcp_m']=tcp.tolist();out['projection_candidates']=[]
for z in [.27,.25,.24,.23,.22]:
 dest=np.r_[np.median(a,axis=0)[:2],z];Tc=T.copy();Tc[:3,3]+=dest-tcp;points=(np.c_[contour3ds,np.ones(len(contour3ds))]@np.linalg.inv(Tc).T)[:,:3];pix=points@k.T;pix=pix[:,:2]/pix[:,2:3];lo=pix.min(0);hi=pix.max(0);out['projection_candidates'].append({'tcp_m':dest.tolist(),'observed_target_projected_bbox_min':lo.tolist(),'observed_target_projected_bbox_max':hi.tolist(),'image_margin_px':float(min(lo[0],lo[1],639-hi[0],479-hi[1])),'assumption':'unchanged camera extrinsic and current tool orientation; projection of currently observed surface only'})
(P/'contour_localization.json').write_text(json.dumps(out,indent=2));np.save(P/'target_surface_cloud.npy',contour3ds);print(json.dumps({k:v for k,v in out.items() if k!='frames'},indent=2))
