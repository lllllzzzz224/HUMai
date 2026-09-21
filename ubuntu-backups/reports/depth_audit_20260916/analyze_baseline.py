import json,numpy as np,cv2
from pathlib import Path
from scipy.spatial.transform import Rotation
P=Path(__file__).parent;B=P/'baseline';c=json.loads((B/'capture.json').read_text());raw={r['stamp_ns']:r for r in c['records']['raw']};ali={r['stamp_ns']:r for r in c['records']['aligned']};stamps=sorted(raw.keys()&ali.keys());rgb=c['records']['rgb'];rgbst=np.array([r['stamp_ns']for r in rgb]);K=np.array(c['camera_info']['raw']['K']).reshape(3,3);C=np.array(c['camera_info']['rgb']['K']).reshape(3,3);tf=c['transforms']['camera_color_optical_frame<-camera_depth_optical_frame'];R=Rotation.from_quat(tf['xyzw']).as_matrix();t=np.array(tf['xyz']);rois=[[175,110,210,140],[365,220,395,250],[385,360,410,390],[125,365,155,395]]
rows=[]
for s in stamps[::10]:
 d=np.load(B/raw[s]['path']);a=np.load(B/ali[s]['path']);yy,xx=np.mgrid[60:450,200:650];z=d[yy,xx]*.001;good=z>0;xx=xx[good];yy=yy[good];z=z[good];q=np.c_[(xx-K[0,2])*z/K[0,0],(yy-K[1,2])*z/K[1,1],z]@R.T+t;u=q[:,0]*C[0,0]/q[:,2]+C[0,2];v=q[:,1]*C[1,1]/q[:,2]+C[1,2]
 ri=int(np.argmin(abs(rgbst-s)));row={'stamp_ns':s,'raw_index':raw[s]['index'],'aligned_index':ali[s]['index'],'rgb_index':rgb[ri]['index'],'rgb_error_ms':float(abs(rgbst[ri]-s)/1e6),'rois':[]}
 for x0,y0,x1,y1 in rois:
  m=(u>=x0)&(u<x1)&(v>=y0)&(v<y1);rv=z[m]*1000;av=a[y0:y1,x0:x1];valid=av[av>0];up=np.rint(u[m]).astype(int);vp=np.rint(v[m]).astype(int);inside=(up>=0)&(up<a.shape[1])&(vp>=0)&(vp<a.shape[0]);sample=a[vp[inside],up[inside]];match=sample>0;diff=sample[match]-rv[inside][match]
  row['rois'].append({'raw_projected_count':len(rv),'raw_median_mm':float(np.median(rv)) if len(rv)else None,'aligned_valid_fraction':float(np.mean(av>0)),'aligned_median_mm':float(np.median(valid))if len(valid)else None,'same_stamp_projected_absdiff_p95_mm':float(np.quantile(abs(diff),.95))if len(diff)else None})
 rows.append(row)
summary=[]
for i,roi in enumerate(rois):
 rr=np.array([r['rois'][i]['raw_median_mm']for r in rows],dtype=float);aa=np.array([r['rois'][i]['aligned_median_mm']for r in rows],dtype=float);good=np.isfinite(rr)&np.isfinite(aa)
 summary.append({'roi':roi,'raw_median_range_mm':[float(np.nanmin(rr)),float(np.nanmax(rr))],'aligned_median_range_mm':[float(np.nanmin(aa)),float(np.nanmax(aa))],'raw_median_p05_p95_mm':np.nanquantile(rr,[.05,.95]).tolist(),'aligned_median_p05_p95_mm':np.nanquantile(aa,[.05,.95]).tolist(),'temporal_correlation':float(np.corrcoef(rr[good],aa[good])[0,1]),'aligned_valid_fraction_range':[min(r['rois'][i]['aligned_valid_fraction']for r in rows),max(r['rois'][i]['aligned_valid_fraction']for r in rows)]})
streams={}
for name,recs in c['records'].items():
 ns=np.array([r['stamp_ns']for r in recs]);delta=np.diff(ns)/1e9;streams[name]={'n':len(recs),'encoding':recs[0]['encoding'],'shape':[recs[0]['height'],recs[0]['width']],'fps':(len(recs)-1)/((ns[-1]-ns[0])/1e9),'max_stamp_gap_s':float(max(delta)),'non_increasing_stamp_count':int(np.sum(delta<=0)),'long_gap_count_gt_50ms':int(np.sum(delta>.05))}
result={'exact_raw_aligned_pairs':len(stamps),'analyzed_every_10th_pair':len(rows),'rgb_error_ms_range':[min(r['rgb_error_ms']for r in rows),max(r['rgb_error_ms']for r in rows)],'streams':streams,'roi_summary':summary,'rows':rows,'logs_n':len(c['logs']),'units':'ROS 16UC1 millimeters based on installed 4.58.1 upstream fix_depth_scale; SDK depth_units not queried'}
(P/'analysis.json').write_text(json.dumps(result,indent=2));im=cv2.imread(str(B/'rgb_0000.jpg'))
for i,(x0,y0,x1,y1)in enumerate(rois):cv2.rectangle(im,(x0,y0),(x1,y1),(0,255,255),2);cv2.putText(im,f'P{i}',(x0,y0-4),0,.5,(0,0,255),1)
cv2.imwrite(str(P/'roi_selection.jpg'),im);print(json.dumps({k:v for k,v in result.items()if k!='rows'}))
