import sys,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path('reports/birdview_alignment_20260916').resolve()))
from local_fk import fk,URDF
before=Path('reports/table_clearance_readonly_20260916/final_stationarity.json')
after=Path('reports/vertical_diagnostic20_20260916/four_step_plan/after_completed_prefix/final_stationarity.json')
def calc(path):
 d=json.loads(path.read_text());ts=[fk(x['read']['data']['arm_state']['joint']) for x in d['samples']];R=Rotation.from_matrix(np.array([t[:3,:3] for t in ts])).mean();tcp=np.mean([(t@np.array([0,0,.138,1]))[:3] for t in ts],axis=0);link=np.mean([t[:3,3] for t in ts],axis=0)
 return tcp,link,R
x,lx,rx=calc(before);y,ly,ry=calc(after);angle=float(np.rad2deg((ry*rx.inv()).magnitude()));drop=float((x[2]-y[2])*1000)
out={'offline_only':True,'before_source':str(before),'after_source':str(after),'urdf_sha256':hashlib.sha256(URDF.read_bytes()).hexdigest(),'before_tcp_mean_m':x.tolist(),'after_tcp_mean_m':y.tolist(),'tcp_delta_mm':((y-x)*1000).tolist(),'link6_delta_mm':((ly-lx)*1000).tolist(),'orientation_change_deg':angle,'maximum_rotation_displacement_at_220mm_mm':float(2*220*np.sin(np.deg2rad(angle)/2)),'completed_descents_total':4,'completed_new_batch_descents':3,'fk_vertical_drop_mm':drop,'manual_before_mm':270,'manual_after_mm':150.6,'manual_drop_mm':119.4,'expected_after_mm_if_same_tip_table_and_rigid_mount':270-drop,'manual_minus_fk_drop_mm':119.4-drop,'interpretation':'Manual difference exceeds joint-FK drop; small attitude difference cannot explain 39mm for tool-scale lever arms. Baseline ruler reading, same fingertip/table reference, vertical measurement, mount/base/table stability and kinematic scale remain unverified. No calibration change can be inferred from these two scalar readings.'}
Path(__file__).with_name('comparison.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
