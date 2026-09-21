import copy,json,textwrap
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from preflight_alignment import P,load_config
D=json.loads((P/'scene_readback_audit.json').read_text())
def obj(d):
 if isinstance(d,dict):return NS(**{k:obj(v) for k,v in d.items()})
 if isinstance(d,list):return [obj(v) for v in d]
 return d
s=(P/'execute_alignment_once.py').read_text();s=textwrap.dedent(s[s.index('  def composed_box'):s.index('  n.current_state_is_valid()')])
n=NS(base='base_link');cfg=load_config(P/'config.yaml');results=[]
for case in ['object_pose','primitive_pose','camera_quaternion_sign','wrong_table_10mm']:
 dd=copy.deepcopy(D)
 t=dd['table'][0]
 if case=='primitive_pose':t['primitive_poses'][0],t['pose']=t['pose'],t['primitive_poses'][0]
 if case=='wrong_table_10mm':t['pose']['position']['z']+=.010
 if case=='camera_quaternion_sign':
  for a in dd['attached']:
   if a['object']['id']=='realsense_camera_guard':
    q=a['object']['pose']['orientation']
    for k in q:q[k]*=-1
 world={t['id']:obj(t)};attached={a['object']['id']:obj(a['object']) for a in dd['attached']};record={}
 try:exec(s);result='ACCEPT'
 except RuntimeError as e:result='REJECT';record['reason']=str(e)
 assert result==('REJECT' if case=='wrong_table_10mm' else 'ACCEPT')
 results.append(dict(case=case,result=result,**record))
(P/'scene_gate_offline_tests.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
