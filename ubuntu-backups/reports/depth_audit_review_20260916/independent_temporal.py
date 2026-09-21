from pathlib import Path
import json,numpy as np
p=Path(__file__).resolve().parent.parent/'depth_audit_20260916/baseline';c=json.loads((p/'capture.json').read_text());out={'units':'ROS stored uint16 units, not independently calibrated distance','rois':[]}
for x,y in [(195,130),(405,275),(395,360)]:
 med=[];coverage=[]
 for r in c['records']['aligned']:
  a=np.load(p/r['path'])[y-3:y+4,x-3:x+4];v=a[a>0];coverage.append(len(v)/a.size)
  med.append(float(np.median(v)) if len(v) else None)
 v=np.array([q for q in med if q is not None]);out['rois'].append({'uv':[x,y],'valid_frames':len(v),'total_frames':len(med),'min':float(v.min()),'max':float(v.max()),'p5_p95':np.percentile(v,[5,95]).tolist(),'median_nonzero_fraction':float(np.median(coverage)),'all_medians':med})
raw={r['stamp_ns'] for r in c['records']['raw']};align={r['stamp_ns'] for r in c['records']['aligned']};out['exact_raw_aligned_pairs']=len(raw&align)
for k,rs in c['records'].items():
 s=np.array([r['stamp_ns'] for r in rs])/1e9;out[k]={'count':len(rs),'rate':float((len(s)-1)/(s[-1]-s[0])),'largest_gap_s':float(np.diff(s).max())}
Path(__file__).with_name('independent_temporal.json').write_text(json.dumps(out,indent=2));print({k:v for k,v in out.items() if k!='rois'});print([{k:v for k,v in r.items() if k!='all_medians'} for r in out['rois']])
