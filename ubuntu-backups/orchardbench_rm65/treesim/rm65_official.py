"""RM-only official_assist boundaries. No apple position enters the strategy."""
from dataclasses import dataclass
from contextlib import contextmanager
import numpy as np
import newton
import warp as wp
from . import robot, rm65
from .rm65_kinematics import RM65Kinematics, pose_from_xyz_quat, pose_error
from .rm65_runtime import ScratchScene
from .rm65_acceptance import EnvironmentClearance
from .rm65_planning import solve_cartesian_segment
from .rm65_control import RATE

MOUNT=(1.219252813007449,.347841569121514,0.,2.8797932657906435)
READY=np.array([-.4461756944656372,1.2990716695785522,-.9880963563919067,
                .2896624803543091,-1.523876428604126,.09278970211744308])

@dataclass(frozen=True)
class Bucket:
    floor: tuple
    yaw: float = 0.

    def __post_init__(self):
        if self.yaw != 0.:raise ValueError('UNSUPPORTED_BUCKET_YAW')

    @property
    def release_point(self):
        # RM wrist extends behind the TCP; use a higher release target, not a
        # different bucket, collision allowance or placement predicate.
        return np.array(self.floor)+[0,0,robot._BUCKET_WALL_H+.31]

    def placed(self, point, *, released, elapsed):
        # Translate the original chassis-local predicate to the bucket frame.
        # Original floor z = CHASSIS_Z + CHASSIS[2]. No new success threshold.
        p=np.asarray(point)-self.floor
        return bool(released and elapsed>=60 and np.isfinite(p).all()
                    and abs(p[0])<robot._BUCKET_HALF+.02
                    and abs(p[1])<robot._BUCKET_HALF+.02
                    and -robot._CHASSIS[2]<p[2]<.6-robot._CHASSIS[2])

    def build(self,b):
        body=-1  # true world geometry: no zero-mass dynamic free body
        def pose(x,y,z):return wp.transform(wp.vec3(*(np.asarray(self.floor)+[x,y,z])),wp.quat_identity())
        cfg=b.ShapeConfig(density=0.,mu=.9,collision_group=3)
        hw,wh,wt=robot._BUCKET_HALF,robot._BUCKET_WALL_H,.012
        b.add_shape_box(body,xform=pose(0,0,.006),label='rm_receiving_bucket/floor',
                        hx=hw+wt,hy=hw+wt,hz=.006,cfg=cfg,color=robot._BUCKET_COLOR)
        for j,(sx,sy,hx,hy) in enumerate(((1,0,wt,hw+wt),(-1,0,wt,hw+wt),(0,1,hw+wt,wt),(0,-1,hw+wt,wt))):
            b.add_shape_box(body,xform=pose(sx*(hw+wt),sy*(hw+wt),wh*.5),label=f'rm_receiving_bucket/wall{j}',
                            hx=hx,hy=hy,hz=wh*.5,cfg=cfg,color=robot._BUCKET_COLOR)

@contextmanager
def bucket_scene(bucket):
    """Process-local construction hook, including scratch copies; restored on exit."""
    original=rm65.build_fixed
    def build(b,rp):
        maps=original(b,rp);bucket.build(b);return maps
    rm65.build_fixed=build
    try:yield
    finally:rm65.build_fixed=original

def expected_contact(a,b,closing,*,target='apple35'):
    pair=frozenset((a,b))
    return bool(pair==frozenset((target,'rm_receiving_bucket')) or closing and pair in (
        frozenset((target,'gripper_left_finger')),frozenset((target,'gripper_right_finger'))))

def hold_allowed(distance,closed,stable):
    return bool(np.isfinite(distance) and distance<=.005 and closed and stable)

def sphere_separation(a,ra,b,rb):
    return float(np.linalg.norm(np.asarray(a)-b)-ra-rb)

def scratch_joints(snapshot,indices,arm,fingers):
    q=np.array(snapshot,copy=True);q[indices]=np.r_[arm,fingers];return q

def measured_closed(q,v,contact_fingers):
    q,v=np.asarray(q),np.asarray(v)
    return bool(np.isfinite(q).all() and np.isfinite(v).all() and max(abs(v))<=.001
                and all(x>=.059 or x>.001 and i in contact_fingers for i,x in enumerate(q)))

class StableWindow:
    def __init__(self):self.count=0
    def update(self,good):
        self.count=self.count+1 if good else 0;return self.count>=45

class BucketClearance(EnvironmentClearance):
    def distance(self,a,b):
        if any(self.n.shape_label[self.gmap[k]].startswith('rm_receiving_bucket/') for k in (a,b)):
            lower=sphere_separation(self.d.geom_xpos[a],self.m.geom_rbound[a],self.d.geom_xpos[b],self.m.geom_rbound[b])
            if lower>.005:return lower
        return super().distance(a,b)

class PerceptionAdapter:
    def __init__(self):self.sample_id=-1;self.center=None;self.records=[]

    def accept(self,s):
        if s.sample_id<=self.sample_id:raise RuntimeError('STALE_SAMPLE')
        if len(s.detections_world)!=1:raise RuntimeError('AMBIGUOUS_OR_MISSING_DETECTION')
        center=np.array(s.detections_world[0].center_world,float)
        if center.shape!=(3,) or not np.isfinite(center).all():raise RuntimeError('INVALID_DETECTION')
        drift=0. if self.center is None else float(np.linalg.norm(center-self.center))
        if drift>=.09:raise RuntimeError('TRACK_ASSOCIATION')
        self.sample_id=s.sample_id;self.center=center.copy()
        self.records.append(dict(sample_id=s.sample_id,time=s.simulation_time,center=center.tolist(),association_m=drift))
        return center.copy()

class ObjectAssociation:
    """Ray-hit ID association only. No apple world positions are read."""
    def __init__(self,r,config):self.r=r;self.config=config;self.index=None
    def accept(self,s):
        if len(s.detections_world)!=1:raise RuntimeError('ASSOCIATION_AMBIGUOUS')
        y,x=map(lambda v:int(round(v)),s.detections_world[0].px)
        hits=s.shape_index[max(0,y-1):y+2,max(0,x-1):x+2].ravel()
        sb=self.r.sim.model.shape_body.numpy();valid=hits[(hits>=0)&(hits<len(sb))].astype(int)
        bodies=set(int(sb[i]) for i in valid)
        apples=list(map(int,self.r.tree.apple_data['apple_body']))
        found=[b for b in bodies if b in apples]
        if len(found)!=1:raise RuntimeError('RAY_OBJECT_ASSOCIATION')
        name=self.r.sim.model.body_label[found[0]].rsplit('/',1)[-1]
        if name!=self.config.apple:raise RuntimeError('WRONG_FIXTURE_ID')
        i=apples.index(found[0])
        parent=int(self.r.tree.apple_data['parent_body'][i])
        if self.r.sim.model.body_label[parent].rsplit('/',1)[-1]!=self.config.stem:
            raise RuntimeError('WRONG_FIXTURE_STEM')
        if self.index is not None and self.index!=i:raise RuntimeError('OBJECT_ID_CHANGED')
        self.index=i;self.r.allowed_pick_index=i

class GripperAdapter:
    def __init__(self,r,association=None,*,grasp_center_local=(0.,0.,0.)):
        center=np.asarray(grasp_center_local,dtype=float)
        if center.shape!=(3,) or not np.isfinite(center).all():raise ValueError('INVALID_GRASP_CENTER')
        self._grasp_center_local=tuple(center)
        self.r=r;self.body=r.audit.body_ids['gripper_tcp']
        self.association=association
        self.hold_attempts=0;self.closing=False
    @property
    def grasp_center_local(self):return self._grasp_center_local
    @property
    def offset(self):return self.grasp_center_local
    def grasp_center_world(self,tcp_pose):
        pose=np.asarray(tcp_pose,dtype=float)
        return pose[:3,3]+pose[:3,:3]@self.grasp_center_local
    def tcp_target_for_fruit(self,detected_center,tcp_pose):
        return np.asarray(detected_center,dtype=float)-np.asarray(tcp_pose)[:3,:3]@self.grasp_center_local
    def state(self):return self.r.controller.read_gripper_state(self.r.sim.state_0)
    def close(self):self.closing=True;self.r.controller.set_gripper_open_fraction(0.)
    def open(self):self.r.controller.set_gripper_open_fraction(1.)
    def hold(self,index,*,distance,closed,stable):
        if self.association is not None and (self.association.index is None or index!=self.association.index):
            raise RuntimeError('HOLD_TARGET_MISMATCH')
        if not hold_allowed(distance,closed,stable):raise RuntimeError('HOLD_GATE')
        self.hold_attempts+=1
        # Same original spring and damping; TCP is the anchor body, no double offset.
        self.r.sim.apples.hold_off=wp.vec3(*self.offset)
        self.r.sim.apples.hold(index,self.body)
    def release(self,index):self.r.sim.apples.release(index);self.open()

class RobotAdapter:
    def __init__(self,r,event,*,target='apple35'):
        self.target=target
        self.r=r;self.event=event;self.kin=RM65Kinematics(device=r.sim.model.device)
        p=r.base_initial;self.base=pose_from_xyz_quat(p[:3],p[3:])
        self.scratch=ScratchScene(r);self.clear=BucketClearance(r)
        self.points=[];self.index=0;self.window=StableWindow();self.goal=None
        self.carried_index=None;self.contact_allowed=False
        self.max_tracking=0.;self.max_fk=0.;self.min_margin=float('inf')

    def actual(self):
        p=self.r.sim.state_0.body_q.numpy()[self.r.audit.body_ids['gripper_tcp']]
        return pose_from_xyz_quat(p[:3],p[3:]/np.linalg.norm(p[3:]))
    def arm(self):return self.r.controller.read_arm_state(self.r.sim.state_0)
    def check(self,q):
        q=np.asarray(q)
        if np.any(q<self.kin.soft_lower) or np.any(q>self.kin.soft_upper):raise RuntimeError('PLAN_LIMIT')
        s=self.scratch;fingers,_=self.r.controller.read_gripper_state(self.r.sim.state_0)
        s.q.assign(scratch_joints(s.snapshot_q,s.qs,q,fingers))
        newton.eval_fk(s.model,s.q,s.qd,s.state,indices=s.indices)
        if self.carried_index is not None:
            # Scratch-only carried geometry follows the declared grip anchor,
            # not a truth-position target. Never write the live apple state.
            bodies=s.state.body_q.numpy();ab=int(self.r.tree.apple_data['apple_body'][self.carried_index])
            tcp=bodies[self.carried_gripper.body]
            bodies[ab,:3]=self.carried_gripper.grasp_center_world(pose_from_xyz_quat(tcp[:3],tcp[3:]))
            s.state.body_q.assign(bodies)
        s.audit.allowed_touch_pairs=self.r.audit.allowed_touch_pairs if self.contact_allowed else set()
        s.audit.check_no_contact();self.clear.check(s.state,'plan');return True
    def allow_target_fingers(self):
        import mujoco
        self.contact_allowed=True;c=self.clear
        keep=[]
        for a,b in c.pairs:
            rb=c.n.body_label[c.sb[c.gmap[a]]].rsplit('/',1)[-1]
            eb=int(c.sb[c.gmap[b]])
            en=c.n.body_label[eb].rsplit('/',1)[-1] if eb>=0 else 'ground'
            if not expected_contact(rb,en,True,target=self.target):keep.append((a,b))
        c.pairs=keep;c.a=np.array([a for a,b in keep]);c.b=np.array([b for a,b in keep])
        c.plane=c.m.geom_type[c.b]==int(mujoco.mjtGeom.mjGEOM_PLANE)
    def carry(self,index,gripper):
        import mujoco
        self.carried_gripper=gripper
        self.carried_index=index;c=self.clear
        ab=int(self.r.tree.apple_data['apple_body'][index])
        apples=[i for i,b in enumerate(c.bids) if b==ab]
        others=[i for i,b in enumerate(c.bids) if b!=ab and b not in c.robot
                and not c.n.shape_label[c.gmap[i]].startswith('rm_receiving_bucket/')]
        c.pairs+= [(a,b) for a in apples for b in others]
        c.a=np.array([a for a,b in c.pairs]);c.b=np.array([b for a,b in c.pairs])
        c.plane=c.m.geom_type[c.b]==int(mujoco.mjtGeom.mjGEOM_PLANE)
    def ready(self):
        q,_=self.arm()
        while True:
            self.check(q)
            if np.array_equal(q,READY):break
            q=q+np.clip(READY-q,-RATE[:6]/60,RATE[:6]/60)
        self.goal=self.base@self.kin.fk(READY);self.points=[READY];self.index=0;self.window=StableWindow()
        self.r.controller.set_arm_targets(READY)
    def plan(self,point,label):
        q,_=self.arm();start=self.base@self.kin.fk(q);goal=start.copy();goal[:3,3]=point
        self.scratch.state.assign(self.r.sim.state_0);self.scratch.snapshot_q=self.r.sim.state_0.joint_q.numpy().copy()
        def slew(a,b):
            for _ in range(10000):
                self.check(a)
                if np.array_equal(a,b):return
                a=a+np.clip(b-a,-RATE[:6]/60,RATE[:6]/60)
            raise RuntimeError('SLEW_BUDGET')
        pts=solve_cartesian_segment(self.kin,np.linalg.inv(self.base)@start,np.linalg.inv(self.base)@goal,q,
            collision_check=self.check,check_slew=slew,timeout_s=600,max_iterations=50,label=label)
        self.points=[p.q_arm for p in pts];self.index=0;self.window=StableWindow();self.goal=goal
        self.event('plan',label=label,seed=q,points=self.points,goal=goal)
        self.r.controller.set_arm_targets(self.points[0])
    def advance(self):
        c=self.r.controller
        if self.index+1<len(self.points) and np.max(abs(c.sent[:6]-c.goal[:6]))<1e-12:
            self.index+=1;c.set_arm_targets(self.points[self.index])
    def settled(self,tolerance=.008):
        return self.window.count>=45

class PlacementScorer:
    """Only scorer reads apple truth coordinates, after release; never returns a target."""
    def __init__(self,r,bucket):self.r=r;self.bucket=bucket
    def score(self,index,released,elapsed):
        if not released or elapsed<60:return False
        body=int(self.r.tree.apple_data['apple_body'][index])
        pos=self.r.sim.state_0.body_q.numpy()[body,:3]
        return self.bucket.placed(pos,released=released,elapsed=elapsed)
