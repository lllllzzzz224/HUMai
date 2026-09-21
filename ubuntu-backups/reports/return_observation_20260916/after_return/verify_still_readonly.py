import sys
sys.path.insert(0,'/home/li/桌面/9月机械臂实验（算法）/reports/birdview_alignment_20260916')
import json,time
from pathlib import Path
import numpy as np
from local_fk import arm,fk
P=Path(__file__).resolve().parent
samples=[]
for _ in range(12):
 r=arm();q=r['data']['arm_state']['joint'];T=fk(np.array(q));samples.append({'read':r,'joint_deg':(np.array(q)/1000).tolist(),'tcp_m':(T@np.array([0,0,.138,1]))[:3].tolist()});time.sleep(.15)
a=np.array([r['joint_deg'] for r in samples]);xyz=np.array([r['tcp_m'] for r in samples]);out={'mode':'read-only actual controller state; no command except get_current_arm_state','samples':samples,'joint_span_deg':np.ptp(a,axis=0).tolist(),'tcp_axis_span_mm':(np.ptp(xyz,axis=0)*1000).tolist(),'duration_s':samples[-1]['read']['end_wall']-samples[0]['read']['start_wall'],'controller_errors':[r['read']['data']['arm_state']['err'] for r in samples]};out['stationary_sampled']=bool(np.max(np.ptp(a,axis=0))<.02);(P/'final_stationarity.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k!='samples'}))
