"""Write-only ground-truth diagnostics. Never returns targets to control."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .rm65_kinematics import pose_from_xyz_quat

def relative_geometry(tcp,apple,center):
    local=tcp[:3,:3].T@(np.asarray(apple)-tcp[:3,3])
    relative=local-np.asarray(center)
    return dict(apple_relative_tcp_local_m=local.tolist(),
        apple_relative_grasp_center_local_m=relative.tolist(),
        hold_anchor_error_m=float(np.linalg.norm(relative)))

class PickDiagnostics:
    def __init__(self,output,r,gripper,target):
        self.r=r;self.gripper=gripper
        ids=list(map(int,r.tree.apple_data['apple_body']))
        self.index=next(i for i,b in enumerate(ids) if r.sim.model.body_label[b].rsplit('/',1)[-1]==target)
        self.apple_body=ids[self.index]
        self.log=(Path(output)/'pick-diagnostics.jsonl').open('x',buffering=1)
        urdf=ET.parse(Path(__file__).resolve().parents[1]/'assets/robots/rm65/rm65_with_gripper.urdf')
        self.names=('gripper_left_finger','gripper_right_finger')
        self.widths=[float(urdf.find(f".//link[@name='{name}']/collision/geometry/box").get('size').split()[1]) for name in self.names]
        self.touch=[dict(substeps=0,events=0,first_time_s=None,peak_force_N=0.) for _ in self.names]
        self.was_touch=[False,False]
    def observe(self,state,stage,contacts,*,phase='post_solver'):
        r=self.r;a=r.sim.apples;i=self.index;g=self.gripper
        poses=state.body_q.numpy();tcp=pose_from_xyz_quat(poses[g.body,:3],poses[g.body,3:])
        center=g.grasp_center_world(tcp);apple=poses[self.apple_body,:3]
        velocity=state.body_qd.numpy()[self.apple_body,:3]
        held=bool(a._held_host[i]);detached=bool(a.detached[i])
        elastic=(2.5 if detached else 1.)*a.hold_k*(center-apple) if held else np.zeros(3)
        damping=-a.hold_c*velocity if held else np.zeros(3)
        q,v=r.controller.read_gripper_state(state)
        points=np.array([poses[r.audit.body_ids[n],:3] for n in self.names])
        # Signed inner-face gap projected onto the common TCP finger-closing axis.
        gap=float((tcp[:3,:3].T@(points[0]-points[1]))[1]-sum(self.widths)/2)
        current=[c for c in reversed(contacts) if c['step']==r.substeps]
        if phase=='post_solver':
            for j,name in enumerate(self.names):
                rows=[c for c in current if name in c['pair'] and r.sim.model.body_label[self.apple_body].rsplit('/',1)[-1] in c['pair']]
                touching=bool(rows);stats=self.touch[j]
                if touching:
                    stats['substeps']+=1;stats['events']+=int(not self.was_touch[j])
                    stats['peak_force_N']=max(stats['peak_force_N'],max(c['force_N'] for c in rows))
                    if stats['first_time_s'] is None:stats['first_time_s']=r.substeps*r.sim.frame_dt/r.sim.substeps
                self.was_touch[j]=touching
        row=dict(step=r.substeps,frame=r.frames,time_s=r.substeps*r.sim.frame_dt/r.sim.substeps,stage=stage,phase=phase,
            truth_usage='diagnostic_only_no_control_return',**relative_geometry(tcp,apple,g.grasp_center_local),
            grasp_center_local=list(g.grasp_center_local),grasp_center_world=center.tolist(),apple_world=apple.tolist(),
            finger_q_m=q.tolist(),finger_velocity_m_s=v.tolist(),finger_goal_m=r.controller.goal[6:].tolist(),
            finger_reference_m=r.controller.sent[6:].tolist(),inner_face_gap_m=gap,
            finger_origin_distance_m=float(np.linalg.norm(points[0]-points[1])),contacts=current,
            finger_apple_contact_summary=[dict(name=n,**s,duration_s=s['substeps']*r.sim.frame_dt/r.sim.substeps) for n,s in zip(self.names,self.touch)],
            held=held,detached=detached,hold_body=int(a._hold_body_host[i]) if held else None,
            hold_anchor=list(a.hold_off) if held else None,
            derived_postsolver_hold_elastic_N=elastic.tolist(),derived_postsolver_hold_damping_N=damping.tolist(),
            derived_postsolver_hold_total_N=(elastic+damping).tolist(),
            applied_presolver_pull_N=float(a._pull.numpy()[i]),applied_presolver_tension_N=float(a._tension.numpy()[i]),
            detach_force_N=float(a.detach_force[i]),pull_counter=int(a._over[i]),tension_counter=int(a._over_t[i]),
            pull_frames_required=a._hyst,tension_frames_required=a._hyst_t,tension_factor=a._tension_factor)
        self.log.write(json.dumps(row)+'\n')
    def close(self):self.log.close()
