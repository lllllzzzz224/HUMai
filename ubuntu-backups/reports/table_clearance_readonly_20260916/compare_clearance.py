import sys,json,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/li/桌面/9月机械臂实验（算法）/reports/birdview_alignment_20260916')
from local_fk import fk,URDF
P=Path(__file__).resolve().parent;s=json.loads((P/'final_stationarity.json').read_text());sample=s['samples'][-1];raw=sample['read']['data']['arm_state']['joint'];T=fk(np.array(raw));tcp=(T@np.array([0,0,.138,1]))[:3]
source=Path('/home/li/桌面/9月机械臂实验（算法）/reports/birdview_return_20260916/after_return/sphere_diagnostic.json');plane=json.loads(source.read_text())['local_table'];a,b,c=plane['coefficients_z_ax_by_c']
def table(p):return a*p[0]+b*p[1]+c
boxes={}
for name,(hx,hy,h) in {'conservative_attachment_160x160x220':(.08,.08,.22),'urdf_approx_80x80x170':(.04,.04,.17)}.items():
 corners=np.array([[x,y,z,1] for x in [-hx,hx] for y in [-hy,hy] for z in [0,h]]);q=(corners@T.T)[:,:3];low=q[np.argmin(q[:,2])];clear=q[:,2]-(a*q[:,0]+b*q[:,1]+c)
 boxes[name]={'lowest_corner_base_m':low.tolist(),'paper_plane_z_at_lowest_corner_m':float(table(low)),'lowest_corner_vertical_to_paper_mm':float((low[2]-table(low))*1000),'minimum_all_corners_vertical_to_paper_mm':float(clear.min()*1000),'lowest_corner_vertical_to_guard_mm':float((low[2]+.055)*1000),'dimensions_m':[hx*2,hy*2,h]}
out={'no_motion_no_gripper_no_scan_no_new_fit':True,'joint_deg':(np.array(raw)/1000).tolist(),'controller_errors':sample['read']['data']['arm_state']['err'],'tcp_base_m':tcp.tolist(),'table_plane_source':str(source),'table_plane_coefficients_z_ax_by_c':[a,b,c],'table_plane_fit_rmse_mm':plane['rmse_mm'],'table_z_at_tcp_m':float(table(tcp)),'tcp_vertical_to_paper_mm':float((tcp[2]-table(tcp))*1000),'tcp_vertical_to_guard_mm':float((tcp[2]+.055)*1000),'guard_top_m':-.055,'boxes':boxes,'urdf_source':str(URDF),'urdf_sha256':hashlib.sha256(URDF.read_bytes()).hexdigest(),'limitations':['tool box lowest corner is not a measured physical finger tip','paper plane is previous local RGBD estimate in base frame conditional on unchanged unverified hand-eye','projecting plane beneath tool corners may extrapolate beyond sampled paper ROIs','all distances are vertical, not normal distances or true contact clearance; ruler measurement of each fingertip remains needed'],'stationary_sampled':s['stationary_sampled'],'joint_span_deg':s['joint_span_deg'],'tcp_axis_span_mm':s['tcp_axis_span_mm']}
(P/'model_clearance.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
