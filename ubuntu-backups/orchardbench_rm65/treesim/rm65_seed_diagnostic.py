"""Bounded seed screening: inherit motion/sensing, prohibit all harvest actions."""
import dataclasses
import numpy as np
from PIL import Image
from .rm65_official_strategy import LocalGraspStrategy

class NaturalAssociation:
    """Identify the sole natural detection by ray ID, never by fruit position."""
    def __init__(self,r,expected=None):
        self.r=r;self.expected=expected;self.index=None;self.name=None
    def accept(self,s):
        if len(s.detections_world)!=1:raise RuntimeError('ASSOCIATION_AMBIGUOUS')
        y,x=(int(round(v)) for v in s.detections_world[0].px)
        hits=s.shape_index[max(0,y-1):y+2,max(0,x-1):x+2].ravel()
        sb=self.r.sim.model.shape_body.numpy()
        valid=hits[(hits>=0)&(hits<len(sb))].astype(int)
        bodies=set(int(sb[i]) for i in valid)
        apples=list(map(int,self.r.tree.apple_data['apple_body']))
        found=[b for b in bodies if b in apples]
        if len(found)!=1:raise RuntimeError('RAY_OBJECT_ASSOCIATION')
        name=self.r.sim.model.body_label[found[0]].rsplit('/',1)[-1]
        index=apples.index(found[0])
        if self.expected is not None and name!=self.expected:raise RuntimeError('REPEAT_TARGET_CHANGED')
        if self.index is not None and self.index!=index:raise RuntimeError('OBJECT_ID_CHANGED')
        self.name=name;self.index=index;self.r.allowed_pick_index=index

class RevalidateOnly(LocalGraspStrategy):
    allowed_states=frozenset(('INIT','READY','DETECT','PREGRASP','REVALIDATE','REVALIDATE_ONLY_DONE'))
    def __init__(self,output,bucket,expected=None,*,config=None):
        if config is None:
            # Historical seed-screen diagnostic only; never a full-run fallback.
            from types import SimpleNamespace
            config=SimpleNamespace(mode='official_assist',apple='apple35',stem='seg294',
                                   pregrasp_offset_tool=(.02,0.,-.03))
        super().__init__(output,bucket,config=config)
        self.expected=expected;self.blocked_calls={};self.captured=[]
    def goto(self,state):
        if state not in self.allowed_states:raise RuntimeError('DIAGNOSTIC_FORBIDDEN_STATE: '+state)
        super().goto(state)
    def forbid(self,name):
        def call(*a,**k):
            self.blocked_calls[name]=self.blocked_calls.get(name,0)+1
            raise RuntimeError('DIAGNOSTIC_FORBIDDEN_ACTION: '+name)
        return call
    def prepare(self,r):
        super().prepare(r)
        r.picking=False  # existing runtime rejects ANY hold or detached fruit
        self.association=NaturalAssociation(r,self.expected)
        self.gripper.close=self.forbid('close');self.gripper.hold=self.forbid('hold')
        self.gripper.release=self.forbid('release')
        r.sim.apples.hold=self.forbid('apples.hold');r.sim.apples.release=self.forbid('apples.release')
        self.save('full-configuration.json',dataclasses.asdict(r.tree.config))
    def closing_authorized(self):return False
    def observe_contacts(self,state):
        start=len(self.contacts);super().observe_contacts(state)
        # No fruit contact is expected anywhere in this diagnostic.
        if any(not row['installation'] for row in self.contacts[start:]):
            raise RuntimeError('DIAGNOSTIC_FORBIDDEN_CONTACT')
    def before_frame(self,r):
        if self.state not in self.allowed_states:raise RuntimeError('DIAGNOSTIC_FORBIDDEN_STATE')
        super().before_frame(r)
    def capture(self,r,s):
        phase='initial' if self.state=='DETECT' else 'near'
        stem=f'{phase}-sample-{s.sample_id:04d}'
        np.savez_compressed(self.output/(stem+'.npz'),depth=s.depth,shape_index=s.shape_index,
            self_mask=s.self_mask,detection_depth=s.detection_depth,T_world_camera=s.T_world_camera)
        ds=[dict(center=d.center_world,radius=d.radius,npix=d.npix,rms=d.rms,px=d.px) for d in s.detections_world]
        self.save(stem+'.json',dict(sample_id=s.sample_id,simulation_time=s.simulation_time,detections=ds))
        gray=(255*(1-np.clip(s.depth/r.camera.range,0,1))).astype(np.uint8)
        rgba=np.dstack((gray,gray,gray,np.full_like(gray,255)))
        Image.fromarray(r.camera.percept.draw_overlay(rgba,s.detections_world,s.detection_depth)).save(self.output/(stem+'.png'))
        self.screenshot(phase+'-view.png');self.captured.append(dict(stage=self.state,sample_id=s.sample_id,artifact=stem))
    def after_camera_sample(self,r,s):
        if self.state not in ('DETECT','REVALIDATE'):return
        self.capture(r,s)
        if self.state=='DETECT':
            return super().after_camera_sample(r,s)  # unchanged trajectory and detection logic
        self.perception.accept(s);self.association.accept(s);self.samples+=1
        self.event('natural_detection',**self.perception.records[-1])
        if self.samples<3:return
        self.goto('REVALIDATE_ONLY_DONE');self.done=True;r.stopped=True
        self.screenshot('near-confirmed.png');self.finish('REVALIDATE_ONLY_PASS')
    def diagnostic_result(self,error=None):
        r=getattr(self,'r',None);a=getattr(self,'association',None)
        record=dict(status='PASS' if self.done and self.state=='REVALIDATE_ONLY_DONE' and not (error or self.failure) else 'FAIL',
            terminal=self.state,last_error=error or self.failure,target=getattr(a,'name',None),
            target_index=getattr(a,'index',None),blocked_calls=self.blocked_calls,captured=self.captured,
            samples=self.perception.records,hold_attempts=getattr(getattr(self,'gripper',None),'hold_attempts',0),
            physics_steps=getattr(r,'substeps',0),base_drift=getattr(r,'max_base_error',0),
            detached=int(r.sim.apples.broken_count) if r is not None else 0,
            grasp_commands=0,close_commands=0,transport_commands=0,release_commands=0)
        if r is not None and getattr(a,'index',None) is not None:
            body=int(r.tree.apple_data['parent_body'][a.index]);record['parent']=r.sim.model.body_label[body]
        self.save('diagnostic-result.json',record)
        return record
