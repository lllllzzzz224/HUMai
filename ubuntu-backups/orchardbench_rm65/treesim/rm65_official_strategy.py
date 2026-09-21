"""Single-fruit sensor-controlled official_assist; no inherited truth servo."""
import json
import math
from pathlib import Path
import numpy as np
import newton
import warp as wp
from PIL import Image
from .rm65_official import (RobotAdapter,GripperAdapter,PerceptionAdapter,ObjectAssociation,
                            PlacementScorer,expected_contact,sphere_separation,measured_closed,READY)
from .rm65_kinematics import pose_error
from .metrics import Metrics

def serial(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

class LocalGraspStrategy:
    def __init__(self,output,bucket,preflight_only=False,*,config=None):
        if config is None or config.mode!='official_assist':raise ValueError('MISSING_OR_INVALID_RUN_CONFIG')
        self.config=config;self.offset=np.array(config.pregrasp_offset_tool,float)
        self.output=Path(output);self.bucket=bucket;self.preflight_only=preflight_only
        self.state='INIT';self.done=False;self.failure=None;self.samples=0;self.contacts=[]
        self.met=Metrics(str(self.output/'metrics.json'))
        self.log=(self.output/'events.jsonl').open('x',buffering=1)
        self.rows=(self.output/'substeps.jsonl').open('x',buffering=1)
        self.contact_log=(self.output/'contacts.jsonl').open('x',buffering=1)
        self.perception=PerceptionAdapter();self.released=False;self.phase_frame=0;self.close_count=0
        self.warning_steps=0;self.warning_events=0;self.in_warning=False;self.finger_contacts=set()
    def save(self,name,data):
        (self.output/name).write_text(json.dumps(data,default=serial,indent=2))
    def event(self,event,**data):
        row=dict(event=event,stage=self.state,frame=getattr(getattr(self,'r',None),'frames',0),**data)
        self.log.write(json.dumps(row,default=serial)+'\n')
    def goto(self,state):
        self.event('transition',target=state);self.state=state;self.phase_frame=self.r.frames;self.samples=0
    def create_gripper(self,r,association=None):
        # Historical diagnostic configurations retain the adapter's zero default.
        return GripperAdapter(r,association,grasp_center_local=getattr(self.config,'grasp_center_local',(0.,0.,0.)))
    def prepare(self,r):
        self.r=r;self.robot=RobotAdapter(r,self.event,target=self.config.apple)
        self.association=ObjectAssociation(r,self.config);self.gripper=self.create_gripper(r,self.association)
        self.save('resolved-gripper.json',dict(fixture_sha256=getattr(self.config,'fixture_sha256',None),
            grasp_center_local=self.gripper.grasp_center_local,hold_anchor=self.gripper.offset,
            hold_body=self.gripper.body,body_label=r.sim.model.body_label[self.gripper.body]))
        self.scorer=PlacementScorer(r,self.bucket) if self.bucket is not None else None
        # Existing runtime picking capability, scoped to this instance only;
        # never instantiate the truth-assisted RMBaselinePicker.
        r.picking=True;r.close_only=self;r.listeners.append(self.after_substep)
        self.forces=newton.Contacts(rigid_contact_max=int(r.sim.solver.mjw_data.naconmax),
            soft_contact_max=0,device=r.sim.model.device,requested_attributes={'force'})
        self.shape_body=r.sim.model.shape_body.numpy();self.robot_bodies=set(r.audit.body_ids.values())
        self.bucket_shapes={i for i,n in enumerate(r.sim.model.shape_label) if n.startswith('rm_receiving_bucket/')}
        self.prepare_bucket(r)
        self.robot.ready()
    def prepare_bucket(self,r):
        if len(self.bucket_shapes)!=5:raise RuntimeError('BUCKET_SHAPE_MAPPING')
        c=self.robot.clear;c.update(r.sim.state_0)
        bg=[i for i,s in enumerate(c.gmap) if s in self.bucket_shapes]
        distances=[]
        for a in bg:
            for b in range(c.m.ngeom):
                if b in bg or b==c.ground:continue
                lower=sphere_separation(c.d.geom_xpos[a],c.m.geom_rbound[a],c.d.geom_xpos[b],c.m.geom_rbound[b])
                d=lower if lower>.005 else c.distance(a,b)
                if d<.005:
                    self.save('bucket-rejected.json',dict(pair=[a,b],distance=d,
                        types=[int(c.m.geom_type[a]),int(c.m.geom_type[b])],
                        labels=[r.sim.model.shape_label[c.gmap[a]],r.sim.model.shape_label[c.gmap[b]]],
                        positions=c.d.geom_xpos[[a,b]],rbound=c.m.geom_rbound[[a,b]]))
                    raise RuntimeError(f'BUCKET_INITIAL_CLEARANCE: {a}/{b}/{d}')
                distances.append(d)
        goal=self.robot.base@self.robot.kin.fk(READY);goal[:3,3]=self.bucket.release_point
        ik=self.robot.kin.solve(np.linalg.inv(self.robot.base)@goal,READY,collision_check=self.robot.check)
        self.save('bucket-ik.json',dict(status=ik.status,candidates=ik.candidates,goal=goal))
        if ik.status!='success':raise RuntimeError('BUCKET_UNREACHABLE: '+ik.status)
        self.robot.scratch.check_slew(READY,ik.q_arm)
        self.save('bucket-preflight.json',dict(floor=self.bucket.floor,release_point=self.bucket.release_point,
            min_scene_clearance_m=min(distances),ik=ik.candidates,robot_only_seed=READY,
            note='robot kinematics, scene geometry; no fruit coordinates used as targets'))
    def closing_authorized(self):return self.gripper.closing
    def observe_contacts(self,state):
        self.last_state=state;r=self.r
        self.finger_contacts=set()
        r.sim.solver.update_contacts(self.forces,state)
        n=int(self.forces.rigid_contact_count.numpy()[0]);s0=self.forces.rigid_contact_shape0.numpy()[:n];s1=self.forces.rigid_contact_shape1.numpy()[:n]
        force=self.forces.force.numpy()[:n];dist=r.sim.solver.mjw_data.contact.dist.numpy()[:n]
        for i,(a,b) in enumerate(zip(s0,s1)):
            ba,bb=int(self.shape_body[a]),int(self.shape_body[b])
            pair=[r.sim.model.body_label[k].rsplit('/',1)[-1] if k>=0 else
                  ('rm_receiving_bucket' if shape in self.bucket_shapes else 'ground') for k,shape in ((ba,a),(bb,b))]
            if ba not in self.robot_bodies and bb not in self.robot_bodies and 'rm_receiving_bucket' not in pair and self.config.apple not in pair:continue
            f=float(np.linalg.norm(force[i,:3]))
            if dist[i]>0 and f==0:continue
            expected=expected_contact(*pair,self.gripper.closing,target=self.config.apple)
            installation=frozenset(pair)==frozenset(('base_link','ground'))
            row=dict(step=r.substeps,time_s=r.substeps/180,stage=self.state,pair=pair,force_N=f,distance_m=float(dist[i]),expected=expected,installation=installation)
            self.contacts.append(row);self.contact_log.write(json.dumps(row)+'\n')
            if not np.isfinite(f) or not (expected or installation):raise RuntimeError('FORBIDDEN_CONTACT: '+str(row))
            if self.config.apple in pair:
                for j,name in enumerate(('gripper_left_finger','gripper_right_finger')):
                    if name in pair:self.finger_contacts.add(j)
    def after_substep(self,state):
        r=self.r;c=r.controller;q,v=c.read_arm_state(state)
        p=state.body_q.numpy()[r.audit.body_ids['gripper_tcp']]
        from .rm65_kinematics import pose_from_xyz_quat
        actual=pose_from_xyz_quat(p[:3],p[3:]/np.linalg.norm(p[3:]))
        fp,fr=pose_error(actual,self.robot.base@self.robot.kin.fk(q))
        ep,er=pose_error(actual,self.robot.base@self.robot.kin.fk(c.sent[:6]))
        margin=float(min(np.min(q-self.robot.kin.soft_lower),np.min(self.robot.kin.soft_upper-q)))
        self.robot.max_tracking=max(self.robot.max_tracking,ep);self.robot.max_fk=max(self.robot.max_fk,fp);self.robot.min_margin=min(self.robot.min_margin,margin)
        warning=ep>.005
        self.warning_steps+=int(warning);self.warning_events+=int(warning and not self.in_warning);self.in_warning=warning
        fingers,fv=c.read_gripper_state(state)
        self.rows.write(json.dumps(dict(step=r.substeps,stage=self.state,q=q,qd=v,tcp=actual,
            reference=c.sent,tracking_m=ep,tracking_rad=er,fk_m=fp,joint_margin=margin,
            fingers=fingers,finger_velocity=fv),default=serial)+'\n')
        if margin<0 or fp>1e-4 or fr>np.deg2rad(.01):raise RuntimeError('LIMIT_OR_FK')
        if ep>.008 or er>np.deg2rad(1):raise RuntimeError(f'TRACKING: {ep}/{er}')
        self.robot.clear.check(state,'execution/'+self.state)
        gp,gr=pose_error(actual,self.robot.goal)
        tolerance=.005 if self.state=='READY' else .008
        complete=self.robot.index==len(self.robot.points)-1 and np.max(abs(c.sent[:6]-c.goal[:6]))<1e-12
        self.robot.window.update(complete and gp<=tolerance and gr<=np.deg2rad(1) and max(abs(v))<=.005)
        if self.state=='CLOSE':
            full=np.max(abs(c.sent[6:]-c.goal[6:]))<1e-12
            good=full and measured_closed(fingers,fv,self.finger_contacts) and self.robot.window.count>=45
            self.close_count=self.close_count+1 if good else 0
    def before_frame(self,r):
        if self.state=='INIT':
            pos=np.array([2.45,-1.55,1.55]);d=np.array([.9,.25,.55])-pos;d/=np.linalg.norm(d)
            r.sim.viewer.set_camera(wp.vec3(*pos),pitch=math.degrees(math.asin(d[2])),yaw=math.degrees(math.atan2(d[1],d[0])))
            self.goto('READY')
        if r.frames-self.phase_frame>7200:raise RuntimeError('PHASE_TIMEOUT: '+self.state)
        self.robot.advance()
    def screenshot(self,name):
        self.r.sim.render();Image.fromarray(self.r.sim.viewer.get_frame().numpy()).save(self.output/name)
    def after_frame(self,r):
        self.met.frame()
        if self.state=='READY' and self.robot.settled(.005):self.goto('DETECT')
        elif self.state=='PREGRASP' and self.robot.settled():self.goto('REVALIDATE')
        elif self.state=='GRASP' and self.robot.settled():self.goto('CLOSE_CONFIRM')
        elif self.state=='CLOSE':
            if self.close_count>=45:
                self.goto('HOLD_CONFIRM')
        elif self.state=='PULL':
            i=self.association.index;self.met.pick_pull(float(r.sim.apples.pull_forces()[i]))
            if r.sim.apples.detached[i]:
                self.met.pick_event('detach');self.event('detached');self.goto('TRANSPORT')
                self.robot.plan(self.bucket.release_point,'TRANSPORT')
            elif self.robot.settled():
                self.pull_distance+=.02
                if self.pull_distance>.32:raise RuntimeError('PULL_LIMIT')
                self.robot.plan(self.pull_origin+self.pull_axis*self.pull_distance,'PULL')
        elif self.state=='TRANSPORT' and self.robot.settled():
            self.gripper.release(self.association.index);self.robot.carried_index=None;self.released=True;self.goto('SCORE')
        elif self.state=='SCORE' and r.frames-self.phase_frame>=60:
            placed=self.scorer.score(self.association.index,self.released,r.frames-self.phase_frame)
            self.finish_metrics(placed)
            if not placed:raise RuntimeError('MISSED_BUCKET')
            self.goto('DONE');self.done=True;r.stopped=True;self.screenshot('released.png');self.finish('PASS')
    def after_camera_sample(self,r,s):
        if self.state not in ('DETECT','REVALIDATE','CLOSE_CONFIRM','HOLD_CONFIRM'):return
        center=self.perception.accept(s);self.association.accept(s);self.samples+=1
        self.event('natural_detection',**self.perception.records[-1])
        if self.samples<3:return
        if self.state=='DETECT':
            self.screenshot('detected.png')
            if self.preflight_only:
                self.goto('PREFLIGHT_DONE');self.done=True;r.stopped=True;self.finish('PREFLIGHT_PASS');return
            self.met.pick_start(self.association.index,center)
            target=center+self.robot.actual()[:3,:3]@self.offset
            self.goto('PREGRASP');self.robot.plan(target,'PREGRASP')
        elif self.state=='REVALIDATE':
            target=self.gripper.tcp_target_for_fruit(center,self.robot.actual())
            self.goto('GRASP');self.robot.plan(target,'GRASP')
        elif self.state=='HOLD_CONFIRM':
            distance=float(np.linalg.norm(self.gripper.grasp_center_world(self.robot.actual())-center))
            fingers,velocity=self.gripper.state()
            self.event('hold_confirmation',sample_id=s.sample_id,distance_m=distance)
            self.gripper.hold(self.association.index,distance=distance,
                closed=measured_closed(fingers,velocity,self.finger_contacts),stable=self.robot.settled())
            self.robot.carry(self.association.index,self.gripper)
            self.met.pick_event('grasp');self.screenshot('closed-held.png');self.goto('PULL')
            self.pull_origin=self.robot.actual()[:3,3].copy();self.pull_axis=-self.robot.actual()[:3,2].copy();self.pull_distance=0.
        else:
            distance=float(np.linalg.norm(self.gripper.grasp_center_world(self.robot.actual())-center))
            self.event('close_distance',distance_m=distance)
            if distance>.005:raise RuntimeError('CLOSE_DISTANCE: '+str(distance))
            self.goto('CLOSE');self.gripper.close()
            r.audit.allowed_touch_pairs={frozenset((self.config.apple,f)) for f in ('gripper_left_finger','gripper_right_finger')}
            self.robot.allow_target_fingers()
    def finish_metrics(self,placed):self.met.pick_end(placed,None if placed else 'missed bucket')
    def finish(self,result):
        self.met.save(self.r.sim)
        metrics=json.loads((self.output/'metrics.json').read_text());metrics['mode']=self.config.mode
        metrics['target_identity']=dict(apple=self.config.apple,stem=self.config.stem,index=self.association.index)
        self.save('metrics.json',metrics)
        self.save('result.json' if result!='FAIL' else 'stop.json',dict(result=result,mode=self.config.mode,
            state=self.state,failure=self.failure,physics_steps=self.r.substeps,bucket_floor=None if self.bucket is None else self.bucket.floor,
            hold_attempts=self.gripper.hold_attempts,detached=int(self.r.sim.apples.broken_count),released=self.released,
            max_tracking_m=self.robot.max_tracking,max_fk_m=self.robot.max_fk,min_joint_margin=self.robot.min_margin,
            base_drift=self.r.max_base_error,warning_steps=self.warning_steps,warning_events=self.warning_events,
            contacts=self.contacts,samples=self.perception.records,metrics=self.met.summary(self.r.sim)))
    def fail(self,exc):
        if self.failure is not None:return
        self.failure=str(exc)
        if hasattr(self,'r'):
            self.r.stopped=True
            state=getattr(self,'last_state',self.r.sim.state_0)
            np.savez_compressed(self.output/'failure-state.npz',joint_q=state.joint_q.numpy(),joint_qd=state.joint_qd.numpy(),body_q=state.body_q.numpy())
            if self.r.sim.viewer is not None:self.screenshot('failure.png')
            if hasattr(self,'gripper'):self.finish('FAIL')
        self.event('failed',error=str(exc))
    def close_files(self):
        for f in (self.log,self.rows,self.contact_log):f.close()
