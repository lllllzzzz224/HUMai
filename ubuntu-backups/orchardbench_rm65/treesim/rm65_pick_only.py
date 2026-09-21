"""Opt-in official_assist pick/hold endpoint. No bucket, transport or release."""
import json
import numpy as np
from .rm65_official_strategy import LocalGraspStrategy

class PickOnlyStrategy(LocalGraspStrategy):
    allowed_states=frozenset(('INIT','READY','DETECT','PREGRASP','REVALIDATE','GRASP',
        'CLOSE_CONFIRM','CLOSE','HOLD_CONFIRM','PULL','DETACH_VERIFY','SAFE_HOLD','PICK_HOLD_DONE'))
    def __init__(self,output,*,config):
        super().__init__(output,None,config=config)
        self.hold_samples=0;self.release_attempts=0;self.hold_calls=[]
        self.diagnostics=None
    def goto(self,state):
        if state not in self.allowed_states:raise RuntimeError('PICK_ONLY_FORBIDDEN_STATE: '+state)
        super().goto(state)
    def forbid_release(self,*a,**k):
        self.release_attempts+=1
        raise RuntimeError('PICK_ONLY_RELEASE_FORBIDDEN')
    def prepare_bucket(self,r):
        if self.bucket is not None or self.bucket_shapes:raise RuntimeError('PICK_ONLY_BUCKET_FORBIDDEN')
    def prepare(self,r):
        super().prepare(r)
        from .rm65_pick_diagnostics import PickDiagnostics
        self.diagnostics=PickDiagnostics(self.output,r,self.gripper,self.config.apple)
        self.gripper.release=self.forbid_release;r.sim.apples.release=self.forbid_release
        hold=self.gripper.hold;close=self.gripper.close
        def recorded_hold(index,**kwargs):
            self.event('hold_call',index=index,body=self.gripper.body,anchor=self.gripper.offset,step=r.substeps)
            result=hold(index,**kwargs)
            row=dict(step=r.substeps,time_s=r.substeps*r.sim.frame_dt/r.sim.substeps,index=index,
                body=int(r.sim.apples._hold_body_host[index]),body_label=r.sim.model.body_label[self.gripper.body],
                anchor=list(r.sim.apples.hold_off),held=int(r.sim.apples._held_host[index]))
            self.hold_calls.append(row);self.event('hold_applied',**row)
            return result
        def recorded_close():
            self.event('close_command',step=r.substeps)
            return close()
        self.gripper.hold=recorded_hold;self.gripper.close=recorded_close
        self.save('pick-only-contract.json',dict(mode='official_assist',task_mode='pick_only',
            bucket_shapes=0,hold_body=self.gripper.body,hold_body_label=r.sim.model.body_label[self.gripper.body],
            hold_anchor=self.gripper.offset,place_status='not_attempted'))
    def verify_detached_hold(self):
        a=self.r.sim.apples;i=self.association.index
        if i is None or not a.detached[i] or not a._held_host[i]:
            raise RuntimeError('DETACH_OR_HOLD_LOST')
    def observe_contacts(self,state):
        try:super().observe_contacts(state)
        finally:
            # Includes the first forbidden-contact substep, even when its gate raises.
            if self.diagnostics is not None:self.diagnostics.observe(state,self.state,self.contacts)
    def close_files(self):
        if self.diagnostics is not None:self.diagnostics.close()
        super().close_files()
    def update_hold_window(self,stable):
        self.verify_detached_hold()
        self.hold_samples=self.hold_samples+1 if stable else 0
    def after_substep(self,state):
        super().after_substep(state)
        if self.state=='SAFE_HOLD':
            self.update_hold_window(self.robot.settled())
    def after_frame(self,r):
        if self.diagnostics is not None and self.state in ('PULL','DETACH_VERIFY','SAFE_HOLD'):
            self.diagnostics.observe(r.sim.state_0,self.state,self.contacts,phase='post_rupture_update')
        if self.state=='PULL':
            i=self.association.index
            self.event('pull_evidence',step=r.substeps,pull_N=float(r.sim.apples.pull_forces()[i]),
                detached=bool(r.sim.apples.detached[i]))
            if r.sim.apples.detached[i]:
                self.met.frame();self.met.pick_pull(float(r.sim.apples.pull_forces()[i]))
                self.verify_detached_hold();self.met.pick_event('detach')
                self.event('detached',mechanism='existing AppleSystem.update force/tension rupture')
                self.goto('DETACH_VERIFY');self.screenshot('detached.png');return
        if self.state in ('DETACH_VERIFY','SAFE_HOLD'):
            self.met.frame();self.verify_detached_hold()
            if self.state=='DETACH_VERIFY' and self.robot.settled():
                # Finish the already collision-checked PULL segment. Do not
                # command a new retreat, home or transport target after detach.
                q,_=self.robot.arm();self.robot.check(q)
                self.save('safe-hold-pose.json',dict(actual_q=q,reference_goal=self.robot.goal))
                self.goto('SAFE_HOLD');self.hold_samples=0
            elif self.state=='SAFE_HOLD' and self.hold_samples*r.sim.frame_dt/r.sim.substeps>=1.:
                self.met.pick_end(False,None)
                self.goto('PICK_HOLD_DONE');self.done=True;r.stopped=True
                self.screenshot('final-held.png');self.finish('PICK_HOLD_PASS')
            return
        previous=self.state
        super().after_frame(r)
        if previous=='GRASP' and self.state=='CLOSE_CONFIRM':self.screenshot('grasp.png')
    def finish(self,result):
        super().finish(result)
        path=self.output/'metrics.json';metrics=json.loads(path.read_text())
        metrics.update(task_mode='pick_only',place_status='not_attempted',hold_calls=self.hold_calls,
            release_attempts=self.release_attempts,safe_hold_s=self.hold_samples*self.r.sim.frame_dt/self.r.sim.substeps)
        metrics['summary']['place_status']='not_attempted'
        for p in metrics['picks']:p['place_status']='not_attempted'
        self.save('metrics.json',metrics)
