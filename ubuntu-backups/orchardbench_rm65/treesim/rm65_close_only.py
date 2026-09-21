"""Stage 3.5 close-only: explicit sequence, sensor targets, no hold or retract."""
from collections import deque
import json
import math
from pathlib import Path
import numpy as np
import newton
import warp as wp
from PIL import Image

from scripts.validate_rm65_camera import ObservationWindow, serial, state_pose, scratch_slew_path
from scripts.validate_rm65_stage35 import unique_detection_center, joint_margin, observation_budget
from .rm65_kinematics import RM65Kinematics, pose_error, target_pose_world_to_base
from .rm65_planning import solve_cartesian_segment
from .rm65_runtime import ScratchScene
from .rm65_acceptance import EnvironmentClearance, arrived
from .rm65_control import RATE

MOUNT = (1.219252813007449, .347841569121514, 0., 2.8797932657906435)
READY = np.array([-.4461756944656372,1.2990716695785522,-.9880963563919067,
                  .2896624803543091,-1.523876428604126,.09278970211744308])


class Sequence:
    ORDER = ('INIT','READY','DETECT','TARGET_CONFIRM','PREGRASP','REVALIDATE',
             'GRASP','CLOSE','CLOSE_ONLY_DONE')

    def __init__(self, pregrasp_only=False):
        self.state = 'INIT'
        self.order = self.ORDER[:5]+('PREGRASP_ONLY_DONE',) if pregrasp_only else self.ORDER

    def advance(self, target):
        if self.state in ('FAILED','CLOSE_ONLY_DONE','PREGRASP_ONLY_DONE'):
            raise RuntimeError('TRANSITION: terminal state')
        if target != 'FAILED' and target != self.order[self.order.index(self.state)+1]:
            raise RuntimeError(f'TRANSITION: {self.state} -> {target}')
        self.state = target


class NaturalTrack:
    """Original picker association radius, with stricter ambiguity/freshness rejection."""
    def __init__(self):
        self.center = None
        self.sample_id = -1

    def add(self, sample_id, detections):
        if sample_id <= self.sample_id: raise RuntimeError('FRESH_SAMPLE_REQUIRED')
        center = unique_detection_center(detections)
        drift = 0. if self.center is None else float(np.linalg.norm(center-self.center))
        if drift >= .09: raise RuntimeError(f'TRACK_ASSOCIATION: {drift}')
        self.center = center.copy(); self.sample_id = sample_id
        return center, drift


def permitted_pair(stage, names):
    pair = frozenset(names)
    return stage == 'CLOSE' and pair in (
        frozenset(('gripper_left_finger','apple35')),
        frozenset(('gripper_right_finger','apple35')))


def close_gate(stage, stable, revalidated, tcp, center):
    distance = float(np.linalg.norm(np.asarray(tcp)-np.asarray(center)))
    if stage != 'GRASP' or not stable or not revalidated or not np.isfinite(distance) or distance > .005:
        raise RuntimeError(f'CLOSE_GATE: stage={stage}, stable={stable}, revalidated={revalidated}, distance={distance}')
    return distance


class TrackingPolicy:
    """Opt-in Stage 3.5 free-space policy; never changes a command or CLOSE gate.

    Duration is sampled occupancy (warning substeps / 180), not a claim about
    exact continuous-time threshold crossings between physics samples.
    """
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.events = []; self.active = None
        self.warning_substeps = 0
        self.maximum_tracking_error_m = 0.
        self.by_stage = {}

    def is_warning(self, stage, position):
        return bool(self.enabled and stage in ('READY','PREGRASP') and position > .005)

    def check(self, stage, position, angle, step):
        if not np.isfinite(position) or not np.isfinite(angle):
            raise RuntimeError('TRACKING_ERROR: nonfinite')
        self.maximum_tracking_error_m = max(self.maximum_tracking_error_m, float(position))
        stats = self.by_stage.setdefault(stage, dict(samples=0, maximum_tracking_error_m=0.,
            maximum_tracking_error_rad=0., warning_substeps=0))
        stats['samples'] += 1
        stats['maximum_tracking_error_m'] = max(stats['maximum_tracking_error_m'], float(position))
        stats['maximum_tracking_error_rad'] = max(stats['maximum_tracking_error_rad'], float(angle))
        warning = self.is_warning(stage, position)
        if warning:
            if self.active is None:
                self.active = dict(stage=stage, first_step=int(step), last_step=int(step),
                    substeps=0, maximum_error_m=0., sample_duration_s=0., open_at_stop=True)
                self.events.append(self.active)
            self.warning_substeps += 1; stats['warning_substeps'] += 1
            self.active['last_step'] = int(step); self.active['substeps'] += 1
            self.active['maximum_error_m'] = max(self.active['maximum_error_m'], float(position))
            self.active['sample_duration_s'] = self.active['substeps']/180.
        elif self.active is not None:
            self.active['open_at_stop'] = False
            self.active['exit_step'] = int(step)
            self.active = None
        hard_limit = .008 if self.enabled and stage in ('READY','PREGRASP') else .005
        if position > hard_limit or angle > np.deg2rad(1.):
            raise RuntimeError(f'TRACKING_ERROR: {position} m / {angle} rad (position limit {hard_limit} m)')
        return warning

    def summary(self):
        return dict(enabled=self.enabled, warning_events=len(self.events),
            warning_substeps=self.warning_substeps, warning_sample_duration_s=self.warning_substeps/180.,
            maximum_tracking_error_m=self.maximum_tracking_error_m,
            events=[dict(e) for e in self.events], by_stage={k:dict(v) for k,v in self.by_stage.items()})


class CloseOnlyProgram:
    def __init__(self, output, *, free_space_tracking=False, pregrasp_only=False):
        self.output = Path(output)
        self.pregrasp_only = pregrasp_only
        self.tracking = TrackingPolicy(enabled=free_space_tracking)
        self.sequence = Sequence(pregrasp_only=pregrasp_only); self.track = NaturalTrack()
        self.gate = ObservationWindow(); self.done = False; self.started = False
        self.revalidated = False; self.close_authorized = False; self.hold_attempts = 0
        self.samples = []; self.acquired = []; self.endpoints = []; self.contact_rows = []
        self.finger_history = deque(maxlen=46); self.last_motion = None
        self.rows = None; self.contacts_log = None; self.events_log = None
        self.motion = []; self.motion_index = 0; self.arrival_count = 0
        self.last_state = None; self.first_failure = None

    @property
    def state(self): return self.sequence.state

    def tracking_summary(self):
        policy = getattr(self, 'tracking', None)
        return policy.summary() if policy is not None else None

    def save(self, name, value):
        with (self.output/name).open('x') as f: json.dump(value,f,indent=2,default=serial)

    def event(self, event, **fields):
        row = dict(event=event,stage=self.state,physics_step=self.r.substeps,**fields)
        self.events_log.write(json.dumps(row,default=serial)+'\n')

    def advance(self, target):
        old = self.state; self.sequence.advance(target)
        self.event('transition',previous=old)
        self.stage_start = self.r.frames

    def prepare(self, r):
        self.r = r; self.last_state = r.sim.state_0
        if r.picking or not r.observation or r.tree.config.seed != 31:
            raise RuntimeError('CLOSE_ONLY_SCOPE')
        if not np.allclose(r.tree.config.robot.stage35_fixed_mount,MOUNT,atol=1e-12,rtol=0):
            raise RuntimeError('CLOSE_ONLY_MOUNT')
        self.rows = (self.output/'substeps.jsonl').open('x',buffering=1)
        self.contacts_log = (self.output/'contacts.jsonl').open('x',buffering=1)
        self.events_log = (self.output/'stages.jsonl').open('x',buffering=1)
        self.kin = RM65Kinematics(device=r.sim.model.device)
        self.base = state_pose(r.base_initial)
        self.clear = EnvironmentClearance(r)
        self.close_clear = EnvironmentClearance(r)
        # Geometry names authorize contacts / scoring only; no position is used as a target.
        apples = list(r.tree.apple_data['apple_body'])
        matches = [int(b) for b in apples if r.sim.model.body_label[int(b)].rsplit('/',1)[-1]=='apple35']
        if len(matches)!=1: raise RuntimeError('TARGET_NAME')
        self.apple = matches[0]; self.apple_index = apples.index(self.apple)
        parent = int(r.tree.apple_data['parent_body'][self.apple_index])
        if r.sim.model.body_label[parent].rsplit('/',1)[-1]!='seg294': raise RuntimeError('TARGET_PARENT')
        keep=[]
        for a,b in self.close_clear.pairs:
            row=self.close_clear.row(a,b,0.,'CLOSE')
            names=[row['robot_link'].rsplit('/',1)[-1],row['environment_object'].rsplit('/',1)[-1]]
            if not permitted_pair('CLOSE',names): keep.append((a,b))
        self.close_clear.pairs=keep
        self.close_clear.a=np.array([a for a,b in keep]); self.close_clear.b=np.array([b for a,b in keep])
        import mujoco
        self.close_clear.plane=self.close_clear.m.geom_type[self.close_clear.b]==int(mujoco.mjtGeom.mjGEOM_PLANE)
        self.force_contacts=newton.Contacts(rigid_contact_max=int(r.sim.solver.mjw_data.naconmax),
            soft_contact_max=0,device=r.sim.model.device,requested_attributes={'force'})
        self.shape_body=r.sim.model.shape_body.numpy()
        self.robot_bodies=set(r.audit.body_ids.values())
        r.sim.apples.hold=self.forbid_hold
        r.close_only=self
        scratch=ScratchScene(r)
        current,_=r.controller.read_arm_state(r.sim.state_0)
        for q in scratch_slew_path(current,READY)[0]:
            joint_margin(self.kin,q); scratch.check(q); self.clear.check(scratch.state,'home_to_ready')
        self.ready_budget=observation_budget(current,READY)
        self.goal_pose=self.base@self.kin.fk(READY)
        r.listeners.append(self.after_substep)
        self.event('prepared',mount=MOUNT,ready=READY,ready_budget=self.ready_budget,
                   minimum=self.clear.minimum,hold_enabled=False,retract_enabled=False)

    def forbid_hold(self, *args, **kwargs):
        self.hold_attempts += 1
        raise RuntimeError('HOLD_FORBIDDEN: close_only')

    def closing_authorized(self):
        return self.state=='CLOSE' and self.close_authorized

    def no_manipulation(self):
        a=self.r.sim.apples
        if self.hold_attempts or np.any(a._held_host) or np.any(a.detached):
            raise RuntimeError('HOLD_OR_DETACH_FORBIDDEN')

    def observe_contacts(self, state):
        """Runs immediately after solver.step, before any rejecting safety gate."""
        self.last_state=state
        self.r.sim.solver.update_contacts(self.force_contacts,state)
        count=int(self.force_contacts.rigid_contact_count.numpy()[0])
        shape0=self.force_contacts.rigid_contact_shape0.numpy()[:count]
        shape1=self.force_contacts.rigid_contact_shape1.numpy()[:count]
        forces=self.force_contacts.force.numpy()[:count]
        distances=self.r.sim.solver.mjw_data.contact.dist.numpy()[:count]
        for i,(s0,s1) in enumerate(zip(shape0,shape1)):
            b0,b1=int(self.shape_body[s0]),int(self.shape_body[s1])
            if b0 not in self.robot_bodies and b1 not in self.robot_bodies: continue
            names=[self.r.sim.model.body_label[b].rsplit('/',1)[-1] if b>=0 else 'ground' for b in (b0,b1)]
            norm=float(np.linalg.norm(forces[i,:3]))
            if distances[i]>0 and norm==0.: continue
            row=dict(physics_step=self.r.substeps,time_s=self.r.substeps/180.,stage=self.state,
                     pair=names,distance_m=float(distances[i]),force_world=forces[i,:3],force_N=norm,
                     expected=permitted_pair(self.state,names))
            self.contacts_log.write(json.dumps(row,default=serial)+'\n'); self.contact_rows.append(row)
            if not np.isfinite(norm): raise RuntimeError('CONTACT_FORCE_NONFINITE')
            if not row['expected'] and frozenset(names)!=frozenset(('base_link','ground')):
                raise RuntimeError('FORBIDDEN_SOLVER_CONTACT: '+str(row))

    def actual_pose(self,state):
        return state_pose(state.body_q.numpy()[self.r.audit.body_ids['gripper_tcp']])

    def stage_arrived(self, position_error, angle_error, velocity):
        if self.tracking.enabled and self.state == 'PREGRASP':
            # Only position is stage-specific. Reuse the original predicate for
            # finite orientation, velocity shape/finiteness and their bounds.
            return bool(np.isfinite(position_error) and position_error <= .008
                        and arrived(0., angle_error, velocity))
        return arrived(position_error, angle_error, velocity)

    def boundary(self):
        r=self.r; c=r.controller
        c.check_state(r.sim.state_0); self.no_manipulation()
        q,v=c.read_arm_state(r.sim.state_0)
        joint_margin(self.kin,q)
        p,angle=pose_error(self.actual_pose(r.sim.state_0),self.goal_pose)
        if not self.stage_arrived(p,angle,v) or not self.gate.ready: raise RuntimeError('UNSTABLE_STAGE_BOUNDARY')
        r.audit.state=r.sim.state_0; r.audit.check_no_contact()
        self.clear.check(r.sim.state_0,'boundary/'+self.state)
        self.event('boundary',q=q,qd=v,tcp_position_error_m=p,tcp_rotation_error_rad=angle)

    def before_frame(self,r):
        if not self.started:
            self.started=True
            pos=np.array([2.45,-1.55,1.55]); direction=np.array([.87,.36,.65])-pos
            direction/=np.linalg.norm(direction)
            r.sim.viewer.set_camera(wp.vec3(*pos),pitch=math.degrees(math.asin(direction[2])),yaw=math.degrees(math.atan2(direction[1],direction[0])))
            r.sim.viewer.renderer.set_title('Stage 3.5 CLOSE ONLY - no hold / no detach')
            self.advance('READY')
            if r.controller.set_arm_targets(READY): raise RuntimeError('UNEXPECTED_CLAMP')
            self.deadline=r.frames+self.ready_budget['ready_deadline_frames']
        if self.state in ('PREGRASP','GRASP') and self.motion:
            c=r.controller
            if np.max(np.abs(c.sent[:6]-c.goal[:6]))<1e-12 and self.motion_index+1<len(self.motion):
                self.motion_index+=1
                q=self.motion[self.motion_index].q_arm
                if c.set_arm_targets(q): raise RuntimeError('UNEXPECTED_CLAMP')
        if self.state in ('DETECT','TARGET_CONFIRM','REVALIDATE'):
            if r.frames-self.stage_start>36: raise RuntimeError('FRESH_DETECTION_TIMEOUT')
        elif r.frames>=self.deadline:
            raise RuntimeError('STAGE_MOTION_TIMEOUT: '+self.state)

    def after_substep(self,state):
        r=self.r; c=r.controller; self.last_state=state
        self.no_manipulation()
        q,v=c.read_arm_state(state); fingers,finger_v=c.read_gripper_state(state)
        margin=joint_margin(self.kin,q)
        actual=self.actual_pose(state); fk=self.base@self.kin.fk(q)
        fp,fr=pose_error(actual,fk)
        sent_pose=self.base@self.kin.fk(c.sent[:6])
        tp,tr=pose_error(actual,sent_pose)
        ep,er=pose_error(actual,self.goal_pose)
        camera=r.camera.binding.world_pose(state.body_q.numpy())
        parent=r.tree.robot_data['body_indices']['gripper_mount'][0]
        expected=state_pose(state.body_q.numpy()[parent])@r.camera.binding.T_parent_camera
        self.gate.update(time_s=r.substeps/180.,target_buffer=r.sim.control.joint_target_q.numpy()[c.ts[:6]],
            effective_target=c.effective_arm_target,sent_target=c.sent[:6],velocity=v,
            actual_camera=camera,expected_camera=expected,safety_ok=True)
        self.last_motion=dict(stage=self.state,physics_step=r.substeps,q=q,qd=v,fingers=fingers,
            finger_velocities=finger_v,closed_fraction=fingers/.06,goal=c.goal.copy(),sent=c.sent.copy(),
            tcp_pose_world=actual,tcp_position_error_m=ep,tcp_rotation_error_rad=er,
            sent_tracking_error_m=tp,sent_tracking_error_rad=tr,fk_error_m=fp,fk_error_rad=fr,
            tracking_soft_warning=self.tracking.is_warning(self.state,tp),
            waypoint_index=self.motion_index if self.motion else None,
            joint_margin=margin,base_pose=state.body_q.numpy()[r.audit.body_ids['base_link']],
            held=int(np.sum(r.sim.apples._held_host)),detached=int(r.sim.apples.broken_count))
        self.rows.write(json.dumps(self.last_motion,default=serial)+'\n')
        if not np.array_equal(self.last_motion['base_pose'],r.base_initial):
            raise RuntimeError('CLOSE_ONLY_BASE_DRIFT')
        if fp>1e-4 or fr>np.deg2rad(.01): raise RuntimeError('ACTUAL_FK_MISMATCH')
        warning = self.tracking.check(self.state,tp,tr,r.substeps)
        if warning:
            self.event('tracking_soft_warning',position_error_m=tp,rotation_error_rad=tr,
                       warning_event=len(self.tracking.events))
        clearance=self.close_clear if self.state=='CLOSE' else self.clear
        clearance.check(state,'execution/'+self.state)
        stable=self.stage_arrived(ep,er,v) and np.max(np.abs(c.sent[:6]-c.goal[:6]))<1e-12
        self.arrival_count=self.arrival_count+1 if stable else 0
        if self.state=='CLOSE':
            complete=np.max(np.abs(c.sent[6:]-c.goal[6:]))<1e-12
            if stable and complete and np.max(np.abs(finger_v))<=.001:
                self.finger_history.append(fingers.copy())
            else: self.finger_history.clear()

    def after_frame(self,r):
        if self.state=='READY' and self.gate.ready and self.arrival_count>=45:
            self.boundary(); self.endpoints.append(self.last_motion.copy()); self.advance('DETECT')
            self.acquired=[]
        elif self.state in ('PREGRASP','GRASP'):
            if self.motion_index==len(self.motion)-1 and self.gate.ready and self.arrival_count>=45:
                self.boundary(); self.endpoints.append(self.last_motion.copy())
                if self.state=='PREGRASP':
                    if self.pregrasp_only:
                        self.advance('PREGRASP_ONLY_DONE'); self.done=True; r.stopped=True
                        r.sim.render()
                        Image.fromarray(r.sim.viewer.get_frame().numpy()).save(self.output/'pregrasp-stable.png')
                        self.save('result.json',dict(result='PASS',mode='pregrasp_only',
                            arrival_position_tolerance_m=.008 if self.tracking.enabled else .005,
                            meaning='stable PREGRASP only; no revalidation or grasp/close acceptance',
                            endpoint=self.last_motion.copy(),endpoints=self.endpoints,samples=self.samples,
                            tracking=self.tracking_summary(),contacts=self.contact_rows,
                            clearance_minimum=getattr(getattr(self,'clear',None),'minimum',None),
                            held=0,detached=0,hold_attempts=self.hold_attempts,retract_commands=0,
                            max_base_drift=r.max_base_error,physics_steps=r.substeps))
                        return
                    self.advance('REVALIDATE'); self.acquired=[]
                else:
                    distance=close_gate(self.state,True,self.revalidated,self.actual_pose(r.sim.state_0)[:3,3],self.track.center)
                    self.event('close_gate',distance_m=distance,detected_center=self.track.center)
                    self.advance('CLOSE'); self.close_authorized=True
                    r.audit.allowed_touch_pairs={frozenset(('gripper_left_finger','apple35')),frozenset(('gripper_right_finger','apple35'))}
                    r.controller.set_gripper_open_fraction(0.)
                    self.deadline=r.frames+240  # Stage 3.2 full stroke 3s + 1s settling.
        elif self.state=='CLOSE' and len(self.finger_history)==46:
            spread=np.ptp(np.asarray(self.finger_history),axis=0)
            if np.max(spread)<=.0005:
                self.no_manipulation(); self.endpoints.append(self.last_motion.copy())
                self.advance('CLOSE_ONLY_DONE'); self.done=True
                r.sim.render()
                Image.fromarray(r.sim.viewer.get_frame().numpy()).save(self.output/'closed.png')
                self.save('result.json',dict(result='PASS',meaning='safe stable closure only; grasp/detachment not proven',
                    endpoints=self.endpoints,samples=self.samples,contacts=self.contact_rows,tracking=self.tracking_summary(),
                    held=0,detached=0,hold_attempts=self.hold_attempts,retract_commands=0,
                    max_base_drift=r.max_base_error,physics_steps=r.substeps))

    def accept_sample(self,s):
        if abs(s.simulation_time-self.r.sim.sim_time)>1e-9: raise RuntimeError('FRESH_SAMPLE_TIME')
        prior=None if self.track.center is None else self.track.center.copy()
        center,drift=self.track.add(s.sample_id,s.detections_world)
        self.acquired.append(center.copy())
        spread=float(np.max(np.linalg.norm(np.asarray(self.acquired)[:,None]-np.asarray(self.acquired)[None,:],axis=-1)))
        if spread>.002: raise RuntimeError('DETECTION_REPEATABILITY')
        # Scoring is downstream of target selection and never alters the selected center.
        truth=self.r.sim.state_0.body_q.numpy()[self.apple,:3]
        error=float(np.linalg.norm(center-truth))
        row=dict(stage=self.state,sample_id=s.sample_id,time=s.simulation_time,center=center,
                 previous_track=prior,association_distance_m=drift,scatter_m=spread,
                 apple35_truth_for_scoring_only=truth,scoring_error_m=error)
        self.samples.append(row); self.save(f'sample-{s.sample_id:05d}.json',row)
        np.savez_compressed(self.output/f'sample-{s.sample_id:05d}.npz',depth=s.depth,
            detection_depth=s.detection_depth,shape_index=s.shape_index,T_world_camera=s.T_world_camera)
        if error>.005: raise RuntimeError('TARGET_SCORING_MISMATCH')

    def after_camera_sample(self,r,s):
        if self.state not in ('DETECT','TARGET_CONFIRM','REVALIDATE') or not self.gate.ready: return
        self.accept_sample(s)
        if self.state=='DETECT': self.advance('TARGET_CONFIRM')
        if len(self.acquired)<3: return
        self.boundary()
        if self.state=='TARGET_CONFIRM':
            self.original_center=self.track.center.copy()
            self.plan_motion('PREGRASP',.08)
        else:
            if np.linalg.norm(self.track.center-self.original_center)>=.09:
                raise RuntimeError('REVALIDATION_ORIGINAL_TRACK_DRIFT')
            self.revalidated=True
            self.plan_motion('GRASP',0.)

    def plan_motion(self,stage,offset):
        r=self.r; q,_=r.controller.read_arm_state(r.sim.state_0)
        pose=self.base@self.kin.fk(q)
        goal=pose.copy(); goal[:3,3]=self.track.center-offset*pose[:3,2]
        scratch=ScratchScene(r); clear=EnvironmentClearance(r)
        arrays=(r.sim.state_0.joint_q,r.sim.state_0.joint_qd,r.sim.state_0.body_q,r.sim.control.joint_target_q)
        before=[a.numpy().copy() for a in arrays]
        def check(x):
            joint_margin(self.kin,x); scratch.check(x); clear.check(scratch.state,'plan/'+stage); return True
        def slew(a,b):
            for x in scratch_slew_path(a,b)[0]: check(x)
        def audit(**data):
            r.sim.render()
            self.event('ik',ik_event=data.pop('event'),requested_stage=stage,**data)
        points=solve_cartesian_segment(self.kin,target_pose_world_to_base(pose,self.base),
            target_pose_world_to_base(goal,self.base),q,collision_check=check,check_slew=slew,
            audit=audit,label=stage,timeout_s=600,max_iterations=50)
        for a,b in zip(arrays,before): np.testing.assert_array_equal(a.numpy(),b)
        path=[q]+[p.q_arm for p in points]
        slew_frames=sum(max(1,math.ceil(float(np.max(np.abs(b-a)/(RATE[:6]/60.))))) for a,b in zip(path,path[1:]))
        self.save(stage.lower()+'-plan.json',dict(seed=q,detected_center=self.track.center.copy(),
            source_sample=self.track.sample_id,start_pose=pose,goal_pose=goal,minimum=clear.minimum,
            slew_frames=slew_frames,settling_frames=600,
            points=[dict(label=p.label,q=p.q_arm,target_pose_base=p.target_pose_base) for p in points]))
        self.advance(stage); self.motion=points; self.motion_index=0; self.arrival_count=0
        self.goal_pose=goal; self.deadline=r.frames+slew_frames+600
        if r.controller.set_arm_targets(points[0].q_arm): raise RuntimeError('UNEXPECTED_CLAMP')

    def fail(self,exc):
        if self.first_failure is not None: return
        stage=self.state; self.first_failure=str(exc)
        if self.state not in ('FAILED','CLOSE_ONLY_DONE','PREGRASP_ONLY_DONE'): self.sequence.advance('FAILED')
        if hasattr(self,'r'):
            self.r.stopped=True  # no further solver steps, target changes or recovery motion
            viewer=getattr(self.r.sim,'viewer',None)
            if viewer is not None:
                try:
                    Image.fromarray(viewer.get_frame().numpy()).save(self.output/'last-rendered-frame.png')
                except Exception as capture_error:
                    self.save('screenshot-error.json',dict(error=str(capture_error)))
        if self.events_log is not None: self.event('failed',failed_stage=stage,error=str(exc))
        if self.last_state is not None:
            np.savez_compressed(self.output/'failure-state.npz',joint_q=self.last_state.joint_q.numpy(),
                joint_qd=self.last_state.joint_qd.numpy(),body_q=self.last_state.body_q.numpy(),body_qd=self.last_state.body_qd.numpy())
        self.save('stop.json',dict(result='FAIL',failed_stage=stage,error=str(exc),last_motion=self.last_motion,
            samples=self.samples,endpoints=self.endpoints,contacts=self.contact_rows,hold_attempts=self.hold_attempts,
            tracking=self.tracking_summary(),
            retract_commands=0,physics_steps=getattr(getattr(self,'r',None),'substeps',0),
            max_base_drift=getattr(getattr(self,'r',None),'max_base_error',None),
            held=int(np.sum(self.r.sim.apples._held_host)) if hasattr(self,'r') else None,
            detached=int(self.r.sim.apples.broken_count) if hasattr(self,'r') else None))

    def close_files(self):
        for f in (self.rows,self.contacts_log,self.events_log):
            if f is not None: f.close()
