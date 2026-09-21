"""Finite Stage 3.4 fixture discovery and independent observation verification.

Truth/shape IDs are used ONLY after the unchanged camera has produced detections.
No controller, scene loader, camera, or detector is implemented here.
"""
import argparse
import json
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy import ndimage
from scripts import validate_rm65_camera as base
from treesim.rm65_kinematics import pose_error

ROOT=Path(__file__).resolve().parents[1]
CANDIDATES=tuple((a,-.8,-.3,0.,b,0.) for a in [-.50,-.45,-.40,-.35,-.30] for b in [-.30,-.20,-.10])
HISTORICAL_MISS=dict(apple='apple35',neighbour='shape365 (apple34 strongly supported by rebuild)',
    result='MISS',migration_error=False,elongation=3.999615325038144,elongation_limit=3.,
    sample=str(ROOT/'output/stage34-observation-readiness/clear-A/sample-00042.npz'),
    limitation='Adjacent fruit merged by original 15 mm depth segmentation; dense fruit recognition not solved')


def component_purity(labels,hits,target_shape):
    labs=sorted(int(x) for x in np.unique(labels[hits==target_shape]) if x>0)
    members={str(l):[int(x) for x in np.unique(hits[labels==l])] for l in labs}
    return dict(pure=bool(len(labs)==1 and members[str(labs[0])]==[int(target_shape)]),
        labels=labs,component_shapes=members,
        target_after_edges=int(np.count_nonzero((hits==target_shape)&(labels>0))),
        component_sizes={str(l):int(np.count_nonzero(labels==l)) for l in labs})


def score_apple(sample,percept,shape_body,body,center,radius,shape,apple):
    """Post-detection scoring only: never calls detect or changes any sample array."""
    T=sample.T_world_camera;c=(center-T[:3,3])@T[:3,:3]
    dot=percept.dirs@c;disc=dot**2-(c@c-radius**2)
    silhouette=(disc>0)&(dot>0);entry=dot-np.sqrt(np.maximum(disc,0))
    hits=sample.shape_index.astype(np.int64);valid_hit=(hits>=0)&(hits<len(shape_body))
    bodies=np.full(hits.shape,-999,dtype=int);bodies[valid_hit]=shape_body[hits[valid_hit]]
    blocked=silhouette&(sample.depth<entry-1e-4)&(bodies!=body)
    total=int(silhouette.sum());visible=int(np.count_nonzero(silhouette&(bodies==body)))
    # Duplicate only the original component-label calculation for independent
    # fixture purity auditing. It never feeds the detector or its sphere fit.
    d=sample.detection_depth;valid=np.isfinite(d)&(d>percept.min_range)&(d<percept.max_range)
    x=np.abs(np.diff(d,axis=1))>percept.edge_jump;y=np.abs(np.diff(d,axis=0))>percept.edge_jump
    edge=np.zeros_like(valid);edge[:,1:]|=x;edge[:,:-1]|=x;edge[1:,:]|=y;edge[:-1,:]|=y
    labels,_=ndimage.label(valid&~edge)
    purity=component_purity(labels,hits,shape)
    matches=[dict(center_world=f.center_world,radius=f.radius,rms=f.rms,npix=f.npix,
        center_error_m=float(np.linalg.norm(f.center_world-center)),radius_error_m=float(abs(f.radius-radius))) for f in sample.detections_world]
    best=min(matches,key=lambda v:v['center_error_m']) if matches else None
    border=bool(silhouette[0].any() or silhouette[-1].any() or silhouette[:,0].any() or silhouette[:,-1].any())
    reasons=[]
    if total<50:reasons.append('LESS_THAN_50_PIXELS')
    if border or c[2]>=0:reasons.append('NOT_FULLY_IN_FRUSTUM')
    if visible!=total or blocked.any():reasons.append('OCCLUDED')
    if not purity['pure']:reasons.append('MIXED_OR_SPLIT_COMPONENT')
    if best is None or best['center_error_m']>.005:reasons.append('NO_ACCURATE_CENTER')
    if best is None or best['radius_error_m']>.005:reasons.append('NO_ACCURATE_RADIUS')
    return dict(apple=apple,body=int(body),shape=int(shape),eligible=not reasons,reasons=reasons,
        silhouette_pixels=total,visible_pixels=visible,purity=purity,best_match=best,
        self_blocked_pixels=int(np.count_nonzero(blocked&sample.self_mask)),
        environment_blocked_pixels=int(np.count_nonzero(blocked&~sample.self_mask)),
        occluder_shapes={str(s):int(np.count_nonzero(blocked&(hits==s))) for s in np.unique(hits[blocked])},
        T_world_camera=T,apple_center_world_for_validation_only=center)


def select_pair(rows):
    ordered=sorted((r for r in rows if r['eligible']),key=lambda r:(r['candidate'],r['apple']))
    if not ordered:return None
    a=ordered[0]
    for b in ordered[1:]:
        if b['apple']!=a['apple'] or b['candidate']==a['candidate']:continue
        p,r=pose_error(np.asarray(a['T_world_camera']),np.asarray(b['T_world_camera']))
        if p>=.01 or r>=np.deg2rad(1):return a,b
    return None


def require_formal_samples(rows,*,phase,process_id,discovery_process_ids):
    if phase!='formal' or process_id in discovery_process_ids:raise ValueError('FORMAL_PROCESS_PROVENANCE')
    if len(rows)!=3:raise ValueError('THREE_SAMPLES_REQUIRED')
    ids=[r['sample_id'] for r in rows];times=[r['simulation_time'] for r in rows]
    if len(set(ids))!=3 or any(b<=a for a,b in zip(ids,ids[1:])) or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('SAMPLES_NOT_FRESH')


def run_pair(pair,runner):
    # Exceptions intentionally propagate; never choose another discovery item.
    return [runner(p) for p in pair]


class FixtureCase(base.ObservationCase):
    def prepare(self,r):
        super().prepare(r)
        self.save('provenance.json',dict(phase=self.args.phase,process_id=os.getpid(),seed=31,
            candidate=self.args.candidate,q=self.args.q,historical_miss=HISTORICAL_MISS))
        m=r.sim.model
        self.save('shape-map.json',dict(body_labels=list(m.body_label),shape_labels=list(m.shape_label),
            shape_body=m.shape_body.numpy(),shape_type=m.shape_type.numpy()))

    def score_all(self,r,sample):
        m=r.sim.model;poses=r.sim.state_0.body_q.numpy();scales=m.shape_scale.numpy();rows=[]
        for b in sorted((int(x) for x in r.tree.apple_data['apple_body']),key=lambda i:m.body_label[i]):
            shapes=np.flatnonzero(self.sb==b)
            if len(shapes)!=1:raise RuntimeError('APPLE_SHAPE_AMBIGUOUS')
            s=int(shapes[0]);name=m.body_label[b].rsplit('/',1)[-1]
            row=score_apple(sample,r.camera.percept,self.sb,b,poses[b,:3],float(scales[s,0]),s,name)
            row.update(candidate=self.args.candidate,phase=self.args.phase,process_id=os.getpid(),
                sample_id=sample.sample_id,simulation_time=sample.simulation_time)
            rows.append(row)
        return rows

    def after_camera_sample(self,r,sample):
        if not self.gate.ready:return
        if abs(sample.simulation_time-r.sim.sim_time)>1e-9:raise RuntimeError('SAMPLE_TIME_MISMATCH')
        parent=r.tree.robot_data['body_indices']['gripper_mount'][0]
        expected=base.state_pose(r.sim.state_0.body_q.numpy()[parent])@r.camera.binding.T_parent_camera
        p,a=pose_error(sample.T_world_camera,expected)
        if p>1e-5 or a>1e-4:raise RuntimeError('SAMPLE_CAMERA_FRAME_MISMATCH')
        rows=self.score_all(r,sample)
        self.save(f'fixture-score-{sample.sample_id:05d}.json',rows)
        if self.args.phase=='discovery':
            np.savez_compressed(self.output/f'sample-{sample.sample_id:05d}.npz',
                depth=sample.depth,detection_depth=sample.detection_depth,shape_index=sample.shape_index,
                self_mask=sample.self_mask,T_world_camera=sample.T_world_camera,sample_id=sample.sample_id,
                time=sample.simulation_time)
            self.save('discovery.json',dict(result='OBSERVED',phase='discovery',process_id=os.getpid(),
                candidate=self.args.candidate,q=self.args.q,rows=rows,motion=self.last_motion,
                physics_steps=r.substeps,frames=r.frames,environment_minimum=self.clearance.minimum,
                installation_min_m=self.clearance.install_min))
            self.done=True
        else:
            own=next(row for row in rows if row['apple']==self.args.apple)
            if self.args.case in ('clear','clear_moved') and not own['eligible']:
                raise RuntimeError('FORMAL_ISOLATED_FIXTURE_INVALID: '+str(own['reasons']))
            super().after_camera_sample(r,sample)
            if self.done:
                require_formal_samples(self.samples,phase=self.args.phase,process_id=os.getpid(),
                    discovery_process_ids=self.args.discovery_pids)
                self.save('formal-provenance-pass.json',dict(process_id=os.getpid(),phase='formal',
                    discovery_process_ids=self.args.discovery_pids,sample_ids=[s['sample_id'] for s in self.samples]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase',choices=['discovery','formal'],required=True)
    parser.add_argument('--candidate',type=int,required=True,choices=range(15))
    parser.add_argument('--apple',default='apple0')
    parser.add_argument('--case',default='clear',choices=['clear','clear_moved','out_of_view','self_occluded','environment_occluded'])
    parser.add_argument('--discovery-pids',nargs='*',type=int,default=[])
    parser.add_argument('--log-dir',type=Path,required=True)
    a=parser.parse_args();a.q=CANDIDATES[a.candidate]
    if a.phase=='formal' and not a.discovery_pids:raise ValueError('DISCOVERY_PROVENANCE_REQUIRED')
    a.log_dir=a.log_dir.resolve();a.log_dir.mkdir(parents=True,exist_ok=False)
    p=FixtureCase(a);viewer=a.log_dir/'viewer-workdir';viewer.mkdir();os.chdir(viewer)
    try:
        base.grow_tree.main(['--robot-model','rm65','--mount','fixed','--rm-control','joint',
            '--rm-camera','--apples','--viewer','gl','--headless','--seed','31','--frames','636'],rm_program=p)
    except Exception as exc:
        steps=getattr(getattr(p,'r',None),'substeps',0)
        # Only known geometric/limit preflight rejections allow discovery to
        # continue. Startup, interface, timeout and runtime failures stop all work.
        unsafe_prefix=('LIMIT:','COLLISION:','STOP: penetration','STOP: finger/',
                       'ENVIRONMENT_CLEARANCE','INSTALLATION_PENETRATION')
        rejected=bool(a.phase=='discovery' and steps==0 and str(exc).startswith(unsafe_prefix))
        p.save('stop.json',dict(result='PREFLIGHT_REJECTED' if rejected else 'FAIL',error=str(exc),
            physics_steps=steps,candidate=a.candidate,process_id=os.getpid(),phase=a.phase,
            last_motion=getattr(p,'last_motion',None)))
        if not rejected:raise

if __name__=='__main__':main()
