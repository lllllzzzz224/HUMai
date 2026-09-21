"""Explicit fixed-fixture revalidation; no downstream harvest states."""
import numpy as np
from .rm65_seed_diagnostic import RevalidateOnly

class FixtureRevalidateOnly(RevalidateOnly):
    def __init__(self,output,bucket,fixture):
        super().__init__(output,bucket,fixture.apple,config=fixture)
        self.stem=fixture.stem
        self.offset=np.array(fixture.pregrasp_offset_tool,float)
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all():
            raise ValueError('INVALID_PREGRASP_OFFSET')
    def prepare(self,r):
        # Identity check only: no apple truth position enters planning.
        names=[r.sim.model.body_label[int(b)].rsplit('/',1)[-1] for b in r.tree.apple_data['apple_body']]
        i=names.index(self.expected)
        parent=int(r.tree.apple_data['parent_body'][i])
        if r.sim.model.body_label[parent].rsplit('/',1)[-1]!=self.stem:
            raise RuntimeError('LOCKED_STEM_CHANGED')
        super().prepare(r)
    def after_camera_sample(self,r,s):
        if self.state!='DETECT':return super().after_camera_sample(r,s)
        self.capture(r,s)
        center=self.perception.accept(s);self.association.accept(s);self.samples+=1
        self.event('natural_detection',**self.perception.records[-1])
        if self.samples<3:return
        self.screenshot('detected.png');self.met.pick_start(self.association.index,center)
        target=center+self.robot.actual()[:3,:3]@self.offset
        self.goto('PREGRASP');self.robot.plan(target,'PREGRASP')
