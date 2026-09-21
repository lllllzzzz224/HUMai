"""One explicit Stage 3.4 observation case through the formal RM joint entry.

No target is selected from detections. An operator supplies fixed joint targets
and an apple name; ground truth is read only here for scoring, never by the
camera/detector. A case PASS is not a complete Stage 3.4 acceptance.
"""
import argparse
import os
import math
from collections import deque
from scipy.spatial.transform import Rotation
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scripts import grow_tree
from scripts.validate_rm65_kinematics import state_pose
from treesim.rm65_kinematics import RM65Kinematics, pose_error
from treesim.rm65_runtime import ScratchScene
from treesim.rm65_acceptance import EnvironmentClearance
from treesim import rm65_control as rc


def serial(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)


def scratch_slew_path(current, target):
    """Build an owned, bounded offline path; no runtime state is accepted.

    Float32 representation tolerance only terminates this scratch loop. It is
    unrelated to dynamic arrival, IK, limits, or collision acceptance thresholds.
    All six original arm rates currently match, so the bound is
    ceil(max(abs(target-start)) / max_step) + 2. The per-axis ratio also remains
    conservative if the existing rate vector ever contains different rates.
    """
    start = np.array(current, dtype=np.float64, copy=True)
    goal = np.array(target, dtype=np.float64, copy=True)
    if start.shape != (6,) or goal.shape != (6,) or not np.isfinite(start).all() or not np.isfinite(goal).all():
        raise ValueError('PREFLIGHT_INPUT: expected two finite six-axis vectors')
    step = np.array(rc.RATE[:6], dtype=np.float64, copy=True) / 60.
    if not np.isfinite(step).all() or np.any(step <= 0):
        raise ValueError('PREFLIGHT_RATE: original rates must be finite and positive')
    scale = max(1., float(np.max(np.abs(start))), float(np.max(np.abs(goal))))
    tolerance = 2. * np.finfo(np.float32).eps * scale
    ratios = np.abs(goal-start)/step
    if not np.isfinite(ratios).all():
        raise ValueError('PREFLIGHT_INPUT: unrepresentable path length')
    bound = math.ceil(float(np.max(ratios))) + 2
    samples = [start.copy()]
    q = start.copy()
    for iteration in range(bound + 1):
        residual = goal-q
        maximum = float(np.max(np.abs(residual)))
        if maximum <= tolerance:
            return tuple(samples), dict(iterations=iteration, iteration_bound=bound,
                final_residual_rad=residual.copy(), final_max_residual_rad=maximum,
                numerical_tolerance_rad=tolerance, termination_reason='numerical_tolerance',
                control_period_s=1./60., max_step_rad=step.copy(), dtype='float64')
        if iteration == bound:
            break
        q = q + np.clip(residual, -step, step)
        samples.append(q.copy())
    raise RuntimeError(f'PREFLIGHT_ITERATION_BOUND: iterations={bound}, residual={maximum}')


class ObservationWindow:
    """Stage 3.4-only readiness: actual camera stability, never command TCP error.

    46 successive substep poses span 45/180 = .25 seconds. Peak-to-peak uses
    maximum pairwise Euclidean position distance and SO(3) angular distance.
    """
    def __init__(self):
        self.history = deque(maxlen=46)
        self.last_time = None
        self.target = None
        self.ready = False
        self.count = 0
        self.audit = {}

    def update(self, *, time_s, target_buffer, effective_target, sent_target,
               velocity, actual_camera, expected_camera, safety_ok=True):
        self.ready = False
        self.audit = {}
        vectors = [np.asarray(x) for x in (target_buffer, effective_target, sent_target, velocity)]
        if not np.isfinite(time_s) or any(x.shape != (6,) or not np.isfinite(x).all() for x in vectors):
            self.history.clear(); self.count = 0
            raise ValueError('OBSERVATION_NONFINITE_OR_INVALID_INPUT')
        if self.last_time is not None and abs((time_s-self.last_time)-1/180.) > 1e-10:
            self.history.clear(); self.count = 0
            raise ValueError('OBSERVATION_CLOCK: require next distinct physics substep')
        self.last_time = time_s
        if not safety_ok:
            self.history.clear(); self.count = 0
            raise RuntimeError('OBSERVATION_SAFETY')
        ep, er = pose_error(actual_camera, expected_camera)
        if ep > 1e-5 or er > 1e-4:
            self.history.clear(); self.count = 0
            raise RuntimeError('CAMERA_FRAME_MISMATCH')
        buffer, goal, sent, v = vectors
        # The buffer must equal the float32 representation actually written by
        # JointController; the float64 slew reference must have reached its goal.
        buffer_done = np.array_equal(buffer, goal.astype(np.float32))
        rounding = 8*np.finfo(np.float64).eps*max(1., float(np.max(np.abs(goal))))
        slew_done = bool(np.all(np.abs(sent-goal) <= rounding))
        moving = bool(np.any(np.abs(v) > .005))
        if self.target is not None and not np.array_equal(goal, self.target):
            self.history.clear()
        self.target = goal.copy()
        if not buffer_done or not slew_done or moving:
            self.history.clear()
        else:
            self.history.append((float(time_s), np.array(actual_camera, dtype=float, copy=True)))
        self.count = len(self.history)
        span = self.history[-1][0]-self.history[0][0] if self.history else 0.
        pe = re = 0.
        if self.history:
            poses = np.asarray([item[1] for item in self.history])
            positions = poses[:,:3,3]
            pe = float(np.max(np.linalg.norm(positions[:,None]-positions[None,:], axis=-1)))
            quats = Rotation.from_matrix(poses[:,:3,:3]).as_quat()
            re = float(np.max(2*np.arccos(np.clip(np.abs(quats@quats.T), 0., 1.))))
        self.ready = bool(self.count == 46 and span >= .25-1e-12 and pe <= .0005 and re <= np.deg2rad(.1))
        self.audit = dict(ready=self.ready, samples=self.count, window_duration_s=span,
            target_buffer_complete=bool(buffer_done), slew_complete=slew_done,
            max_actual_velocity_rad_s=float(np.max(np.abs(v))),
            position_peak_to_peak_m=pe, rotation_peak_to_peak_rad=re,
            camera_frame_position_error_m=ep, camera_frame_rotation_error_rad=er)
        return self.ready


class ObservationCase:
    def __init__(self, args):
        self.args = args
        self.output = args.log_dir
        self.done = False
        self.active = False
        self.gate = ObservationWindow()
        self.ready_time = None
        self.samples = []
        self.max_speed = 0.

    def save(self, name, data):
        with (self.output/name).open('x') as f:
            json.dump(data, f, indent=2, default=serial)

    def prepare(self, r):
        self.r = r
        self.q = np.asarray(self.args.q, float)
        self.kin = RM65Kinematics(device=r.sim.model.device)
        self.target_tcp = state_pose(r.base_initial) @ self.kin.fk(self.q)
        names = r.sim.model.body_label
        matches = [i for i in r.tree.apple_data['apple_body']
                   if names[int(i)].rsplit('/',1)[-1] == self.args.apple]
        if len(matches) != 1:
            raise ValueError('APPLE_NAME: missing or ambiguous validation apple')
        self.apple = int(matches[0])
        self.sb = r.sim.model.shape_body.numpy()
        shapes = np.flatnonzero(self.sb == self.apple)
        if len(shapes) != 1:
            raise ValueError('APPLE_SHAPE: sphere mapping is ambiguous')
        self.radius = float(r.sim.model.shape_scale.numpy()[shapes[0], 0])
        live_arrays = (r.sim.state_0.joint_q, r.sim.state_0.joint_qd,
                       r.sim.state_0.body_q, r.sim.state_0.body_qd, r.sim.control.joint_target_q)
        before = [a.numpy().copy() for a in live_arrays]
        scratch = ScratchScene(r)
        self.clearance = EnvironmentClearance(r)
        current, _ = r.controller.read_arm_state(r.sim.state_0)
        path, termination = scratch_slew_path(current, self.q)
        for q in path:
            scratch.check(q)
            self.clearance.check(scratch.state, 'preflight/'+self.args.case)
        for a,b in zip(live_arrays,before): np.testing.assert_array_equal(a.numpy(),b)
        assert r.substeps == 0
        self.save('preflight.json',dict(seed=31,case=self.args.case,q=self.q,
            apple_name=names[self.apple],apple_body_resolved=self.apple,radius=self.radius,
            minimum=self.clearance.minimum,installation_min_m=self.clearance.install_min,
            physics_steps=0,live_state_and_targets_unchanged=True,
            discrete_clearance_only=True, scratch_termination=termination))
        r.listeners.append(self.after_substep)

    def before_frame(self,r):
        if not self.active:
            r.controller.set_arm_targets(self.q)
            self.active=True

    def after_substep(self,state):
        r=self.r
        q,v=r.controller.read_arm_state(state)
        if np.any(q<r.controller.limits['lower'][:6]) or np.any(q>r.controller.limits['upper'][:6]):
            raise RuntimeError('HARD_LIMIT')
        self.clearance.check(state,'execution/'+self.args.case)
        actual=state_pose(state.body_q.numpy()[r.audit.body_ids['gripper_tcp']])
        fk=state_pose(r.base_initial)@self.kin.fk(q)
        fp,fr=pose_error(actual,fk)
        if fp>1e-4 or fr>np.deg2rad(.01): raise RuntimeError('ACTUAL_FK_MISMATCH')
        ep,er=pose_error(actual,self.target_tcp)
        # Read the current physical parent every substep, not the camera's
        # previous 60 Hz cached pose or a commanded/ideal TCP transform.
        poses = state.body_q.numpy()
        binding = r.camera.binding
        camera_actual = binding.world_pose(poses)
        parent = r.tree.robot_data['body_indices']['gripper_mount'][0]
        expected_camera = state_pose(poses[parent]) @ binding.T_parent_camera
        buffer = r.sim.control.joint_target_q.numpy()[r.controller.ts[:6]]
        ready = self.gate.update(time_s=r.substeps/180., target_buffer=buffer,
            effective_target=r.controller.effective_arm_target,
            sent_target=r.controller.sent[:6], velocity=v,
            actual_camera=camera_actual, expected_camera=expected_camera, safety_ok=True)
        self.max_speed=max(self.max_speed,float(np.max(np.abs(v))))
        self.last_motion=dict(q=q,qd=v,joint_error=r.controller.goal[:6]-q,
                              raw_request=r.controller.raw_arm_request.copy(),
                              effective_target=r.controller.effective_arm_target.copy(),
                              target_buffer=buffer, sent_target=r.controller.sent[:6].copy(),
                              tcp_position_error_m=ep,tcp_rotation_error_rad=er,
                              T_world_camera=camera_actual, observation_window=self.gate.audit.copy(),
                              settled_substeps=self.gate.count,physics_step=r.substeps)
        if self.samples and not ready:
            raise RuntimeError('OBSERVATION_LOST_STABILITY')
        if ready and self.ready_time is None:
            self.ready_time = r.substeps/180.
            self.save('observation-ready.json',dict(simulation_time=self.ready_time,
                physics_steps=r.substeps,motion=self.last_motion))

    def after_frame(self,r):
        if r.frames>=600 and self.ready_time is None:
            raise RuntimeError('TIMEOUT: stable observation not reached within 10 seconds')
        # Once ready, three samples need at most three camera periods. The
        # readiness deadline remains 10 s; this is only bounded acquisition time.
        if self.ready_time is not None and r.sim.sim_time-self.ready_time > 3*r.camera.every/60.+1e-9:
            raise RuntimeError('TIMEOUT: three distinct observation samples not acquired')

    def after_camera_sample(self,r,sample):
        if not self.gate.ready: return
        if self.samples and sample.sample_id <= self.samples[-1]['sample_id']:
            raise RuntimeError('REUSED_SAMPLE')
        if abs(sample.simulation_time-r.sim.sim_time)>1e-9:
            raise RuntimeError('SAMPLE_TIME_MISMATCH')
        poses = r.sim.state_0.body_q.numpy()
        parent = r.tree.robot_data['body_indices']['gripper_mount'][0]
        expected = state_pose(poses[parent]) @ r.camera.binding.T_parent_camera
        frame_p, frame_r = pose_error(sample.T_world_camera, expected)
        if frame_p>1e-5 or frame_r>1e-4:
            raise RuntimeError('SAMPLE_CAMERA_FRAME_MISMATCH')
        if r.sim.viewer.picking_enabled: raise RuntimeError('OBSERVATION_MOUSE_FORCE_ENABLED')
        center=r.sim.state_0.body_q.numpy()[self.apple,:3].copy()
        T=sample.T_world_camera
        c=(center-T[:3,3])@T[:3,:3]
        dirs=r.camera.percept.dirs
        dot=dirs@c
        disc=dot**2-(c@c-self.radius**2)
        silhouette=(disc>0)&(dot>0)
        entry=dot-np.sqrt(np.maximum(disc,0))
        hits=sample.shape_index.astype(np.int64)
        valid=hits<len(self.sb)
        hit_body=np.full(hits.shape,-999,dtype=int)
        hit_body[valid]=self.sb[hits[valid]]
        own=sample.self_mask
        blocked=silhouette&(sample.depth<entry-1e-4)&(hit_body!=self.apple)
        total=int(silhouette.sum())
        visible=int(np.count_nonzero(silhouette&(hit_body==self.apple)))
        nb_self=int(np.count_nonzero(blocked&own))
        nb_env=int(np.count_nonzero(blocked&~own))
        detections=[dict(center_world=d.center_world,radius=d.radius,rms=d.rms,npix=d.npix,
                        center_error_m=float(np.linalg.norm(d.center_world-center)),
                        radius_error_m=abs(d.radius-self.radius)) for d in sample.detections_world]
        best=min(detections,key=lambda d:d['center_error_m']) if detections else None
        obstacles={r.sim.model.body_label[b] if b>=0 else 'ground':int(np.count_nonzero(blocked&(hit_body==b)))
                   for b in np.unique(hit_body[blocked])}
        row=dict(source=sample.source,sample_id=sample.sample_id,simulation_time=sample.simulation_time,
                 T_world_camera=T,apple_center_world_for_validation_only=center,
                 apple_radius_m=self.radius,silhouette_pixels=total,visible_pixels=visible,
                 self_blocked_pixels=nb_self,environment_blocked_pixels=nb_env,
                 occluders=obstacles,detections=detections,best_match=best,
                 self_pixels=int(own.sum()),motion=self.last_motion,
                 sample_frame_position_error_m=frame_p,sample_frame_rotation_error_rad=frame_r)
        self.samples.append(row)
        self.save(f'sample-{sample.sample_id:05d}.json',row)
        np.savez_compressed(self.output/f'sample-{sample.sample_id:05d}.npz',
             depth=sample.depth,detection_depth=sample.detection_depth,shape_index=sample.shape_index,
             self_mask=sample.self_mask,T_world_camera=T,sample_id=sample.sample_id,time=sample.simulation_time)
        kind=self.args.case
        if kind in ('clear','clear_moved'):
            if total<14 or visible!=total: raise RuntimeError('FIXTURE_INVALID: clear is not fully visible at actual camera pose')
            if best is None or best['center_error_m']>.005: raise RuntimeError('CENTER_ERROR_OR_MISSING_DETECTION')
            if best['radius_error_m']>.005: raise RuntimeError('RADIUS_ERROR')
        else:
            if kind=='out_of_view' and total!=0: raise RuntimeError('FIXTURE_INVALID: apple not outside actual view')
            if kind=='self_occluded' and (total<14 or nb_self!=total): raise RuntimeError('FIXTURE_INVALID: incomplete actual RM occlusion')
            if kind=='environment_occluded' and (total<14 or nb_env!=total): raise RuntimeError('FIXTURE_INVALID: incomplete actual environment occlusion')
            if best is not None and best['center_error_m']<.05: raise RuntimeError('FALSE_TARGET_DETECTION')
        if len(self.samples)==3:
            if len({s['sample_id'] for s in self.samples})!=3: raise RuntimeError('REUSED_SAMPLE')
            scatter=None
            if kind in ('clear','clear_moved'):
                centers=np.asarray([s['best_match']['center_world'] for s in self.samples])
                scatter=float(np.max(np.linalg.norm(centers[:,None]-centers[None,:],axis=-1)))
                if scatter>.002: raise RuntimeError('REPEATABILITY')
            self.save('result.json',dict(case=kind,result='PASS',stage34_complete=False,
                samples=self.samples,center_scatter_m=scatter,frames=r.frames,substeps=r.substeps,
                environment_minimum=self.clearance.minimum,maximum_joint_speed=self.max_speed,
                detached=int(r.sim.apples.broken_count),held=int(r.sim.apples._held_host.sum())))
            self.done=True


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',required=True,choices=['clear','clear_moved','out_of_view','self_occluded','environment_occluded'])
    parser.add_argument('--apple',required=True,help='Exact original apple body name, e.g. apple35')
    parser.add_argument('--q',required=True,type=float,nargs=6)
    parser.add_argument('--log-dir',required=True,type=Path)
    a=parser.parse_args(argv)
    a.log_dir=a.log_dir.resolve();a.log_dir.mkdir(parents=True,exist_ok=False)
    p=ObservationCase(a)
    # Viewer ini/cache files belong to this run, never the repository root.
    original_directory = Path.cwd()
    viewer_directory = a.log_dir/'viewer-workdir'
    viewer_directory.mkdir()
    try:
        os.chdir(viewer_directory)
        grow_tree.main(['--robot-model','rm65','--mount','fixed','--rm-control','joint',
            '--rm-camera','--apples','--viewer','gl','--headless','--seed','31','--frames','636'],rm_program=p)
    except Exception as exc:
        p.save('stop.json',dict(result='FAIL',case=a.case,error=str(exc),
                               category='FIXTURE_INVALID' if str(exc).startswith('FIXTURE_INVALID') else type(exc).__name__,
                               samples=p.samples,
                               last_motion=getattr(p,'last_motion',None),
                               physics_steps=getattr(getattr(p,'r',None),'substeps',0),
                               frames=getattr(getattr(p,'r',None),'frames',0)))
        raise
    finally:
        os.chdir(original_directory)


if __name__=='__main__': main()
