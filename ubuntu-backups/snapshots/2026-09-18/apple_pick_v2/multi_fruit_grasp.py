#!/usr/bin/env python3
"""Opt-in same-class rolling-target runner. Default: planning only, return placement."""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
from math import isfinite
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from single_apple_full_grasp import (SingleAppleFullGrasp, FixedScanVerify, AppleTarget,
    prepare_round, apply_visual_xy_correction, load_config, DEFAULT_CONFIG,
    ToolRs485ModbusGripper, scan_arrival_tolerance)
from multi_fruit_state import TargetManager, validate_snapshot
from multi_fruit_runtime import Budget, HardwareFault, Interlock, PhaseLog, Runner
from multi_fruit_transport import GuardedTransport


def same_detections(before,after,tolerance=.008):
    if len(before)!=len(after): return False
    used=set()
    for d in before:
        matches=[i for i,e in enumerate(after) if i not in used and d['class_id']==e['class_id'] and
                 np.linalg.norm(np.asarray(d['center_xyz_m'])-e['center_xyz_m'])<=tolerance and
                 abs(d['radius_m']-e['radius_m'])<=.005]
        if len(matches)!=1: return False
        used.add(matches[0])
    return True


def geometry_signature(scene):
    scene=deepcopy(scene)
    def encoded(value):
        if hasattr(value,'header'):
            value.header.stamp.sec=value.header.stamp.nanosec=0
        return json.dumps(message_to_ordereddict(value),sort_keys=True)
    for attached in scene.robot_state.attached_collision_objects:
        attached.object.header.stamp.sec=attached.object.header.stamp.nanosec=0
    return dict(world={obj.id:encoded(obj) for obj in scene.world.collision_objects},
                attached={obj.object.id:encoded(obj) for obj in scene.robot_state.attached_collision_objects},
                acm=encoded(scene.allowed_collision_matrix),octomap=encoded(scene.world.octomap),
                transforms=sorted(encoded(t) for t in scene.fixed_frame_transforms))


def write_log(path,value):
    temporary=path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8')
    temporary.replace(path)


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--target',choices=['apple','orange'],default='orange')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--operator-confirmed',action='store_true')
    parser.add_argument('--workspace-clear-topic',default='/apple_pick_v2/workspace_clear')
    parser.add_argument('--interlock-source',default='')
    parser.add_argument('--interlock-max-age-s',type=float,default=.5)
    parser.add_argument('--planning-budget-s',type=float,default=2.)
    parser.add_argument('--max-capture-age-s',type=float,default=1.)
    parser.add_argument('--stable-frames',type=int,default=2)
    parser.add_argument('--empty-frames',type=int,default=2)
    parser.add_argument('--empty-interval-s',type=float,default=.05)
    parser.add_argument('--disable-visual-xy-correction',action='store_true')
    parser.add_argument('--replay',action='store_true',help='deterministic three-fruit software replay, no ROS node')
    parser.add_argument('--output',type=Path,default=Path('/tmp/multi_fruit_session.json'))
    parser.add_argument('--max-rounds',type=int,default=0,help='0: keep listening for scene updates')
    args=parser.parse_args(argv)
    for name in ('planning_budget_s','interlock_max_age_s','max_capture_age_s','empty_interval_s'):
        if not isfinite(getattr(args,name)) or getattr(args,name)<=0: parser.error(f'{name} must be positive and finite')
    if args.stable_frames<2 or args.empty_frames<2 or args.max_rounds<0: parser.error('stable-frames >= 2 and max-rounds >= 0 required')
    if args.execute and (args.replay or not args.operator_confirmed or not args.interlock_source):
        parser.error('--execute requires --operator-confirmed and --interlock-source; --replay cannot execute')
    return args


class GuardedGripper(ToolRs485ModbusGripper):
    """Use the existing register driver; guard each publication and wait."""
    def __init__(self,node):
        super().__init__(node)
        for name in ('rs485_pub','voltage_pub','write_pub'):
            publisher=getattr(self,name)
            class Publisher:
                def __init__(self,raw): self.raw=raw
                def __getattr__(self,key): return getattr(self.raw,key)
                def publish(self,message):
                    node.guard()
                    self.raw.publish(message)
            setattr(self,name,Publisher(publisher))
    def _wait_for_subscription(self,publisher,label,timeout_s):
        if not self.node.spin_until(lambda:publisher.get_subscription_count()>=1,timeout_s,label):
            raise HardwareFault(f'gripper subscriber unavailable: {label}')
    def _wait_result(self,key,label,timeout_s):
        if not self.node.spin_until(lambda:key in self.results,timeout_s,label) or not self.results.get(key):
            raise HardwareFault(f'gripper command acknowledgement unconfirmed: {label}')


class MultiFruitNode(SingleAppleFullGrasp):
    def __init__(self,cfg,args,manager,log):
        # Deliberately skip SingleAppleFullGrasp.__init__: no gripper publishers in plan-only mode.
        FixedScanVerify.__init__(self,cfg)
        self.args,self.manager,self.log=args,manager,log
        self.events=[]
        self.gripper=None
        self.scan_stationary_since=float("inf")
        self.stationary_anchor=None
        self.joint_source_stamp=0.
        self.active=False
        self.arming=False
        self.scene_changed=False
        self.snapshot_detections=[]
        self.target=None
        self.cached=[]
        self.neighbor_ids=set()
        self.interlock=Interlock(args.interlock_source,args.interlock_max_age_s)
        self.transport=GuardedTransport(pump=lambda seconds:rclpy.spin_once(self,timeout_sec=seconds),
                                        guard=self.guard,log=log)
        for name,label in [('ik_client','planning_ik'),('cartesian_client','planning_cartesian'),
                           ('validity_client','state_collision'),('scene_client','scene_apply')]:
            setattr(self,name,self.transport.service(getattr(self,name),label))
        self.move_client=self.transport.action(self.move_client,'planning_ompl')
        self.execute_client=self.transport.action(self.execute_client,'motion_action',
                                                  timeout_s=float(cfg['safety']['motion_timeout_s']))
        self.get_scene_client=self.transport.service(self.create_client(GetPlanningScene,'/get_planning_scene'),'scene_read')
        self.create_subscription(String,'/apple_pick_v2/fruit_candidates',self.on_candidates,10)
        self.create_subscription(String,args.workspace_clear_topic,self.on_clear,10)
        self.create_subscription(PlanningScene,'/monitored_planning_scene',self.on_scene,10)

    def ros_now(self): return self.get_clock().now().nanoseconds*1e-9

    def on_joint(self,message):
        super().on_joint(message)
        if not hasattr(self,'stationary_anchor') or not all(n in self.joints for n in self.JOINT_NAMES):
            return
        values=self.current_joint_vector()
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.joint_source_stamp=stamp
        moving=(self.stationary_anchor is None or np.max(np.abs(values-self.stationary_anchor))>.002 or
                any(abs(v)>.01 for v in message.velocity))
        if moving:
            self.stationary_anchor=values.copy()
            self.scan_stationary_since=self.ros_now()

    def on_candidates(self,message):
        if self.active:
            return
        with self.log.phase('perception_receive') as span:
            try:
                value=json.loads(message.data)
                capture=float(value['capture_stamp_s'])
                if (time.monotonic()-self.joint_rx_monotonic>1. or
                        not 0<=self.ros_now()-self.joint_source_stamp<=1. or
                        capture<self.scan_stationary_since+.1 or capture<=self.manager.boundary or
                        np.max(np.abs(self.current_joint_vector()-self.scan_vector()))>scan_arrival_tolerance(self.cfg)):
                    span['accepted']=False
                    return
                accepted=self.manager.update(value,self.ros_now())
                if accepted:
                    self.snapshot_detections=validate_snapshot(value,self.base)[1]
                span.update(capture_stamp_s=value.get('capture_stamp_s'),
                            source_model_duration_s=value.get('source_model_duration_s'),
                            source_localization_duration_s=value.get('source_localization_duration_s'),accepted=accepted)
            except (ValueError,TypeError,KeyError):
                self.manager.valid=False

    def on_clear(self,message):
        try: value=json.loads(message.data)
        except (ValueError,TypeError): value={}
        self.interlock.update(value,self.ros_now(),time.monotonic())
        if not self.safe():
            self.manager.motion_boundary(self.ros_now())
        if (self.active or self.arming) and not self.safe():
            self.transport.fault='workspace-clear lost; stop unconfirmed'
            self.transport.cancel()

    def on_scene(self,message):
        if not self.active or not hasattr(self,'scene_reference'): return
        received=geometry_signature(message); reference=self.scene_reference
        changed=False
        for kind in ('world','attached'):
            changed |= any(reference[kind].get(key)!=value for key,value in received[kind].items())
            if not message.is_diff: changed |= set(received[kind])!=set(reference[kind])
        if message.allowed_collision_matrix.entry_names:
            changed |= received['acm']!=reference['acm']
        if message.world.octomap.octomap.data:
            changed |= received['octomap']!=reference['octomap']
        if message.fixed_frame_transforms:
            changed |= received['transforms']!=reference['transforms']
        if changed:
            self.scene_changed=True
            self.transport.fault='planning-scene geometry changed during actuation; stop unconfirmed'
            self.transport.cancel()

    def safe(self): return self.interlock.safe(self.ros_now(),time.monotonic())

    def guard(self):
        if not rclpy.ok(): raise HardwareFault('ROS shutdown; stop unconfirmed')
        if getattr(self,'transport',None) and self.transport.fault: raise HardwareFault(self.transport.fault)
        if self.active or self.arming:
            if not self.safe(): raise HardwareFault('fresh healthy workspace-clear interlock required')
        if self.active and self.scene_changed: raise HardwareFault('scene changed during execution')
        source_stamp=getattr(self,'joint_source_stamp',0.)
        if not isfinite(source_stamp) or source_stamp<=0 or not 0<=self.ros_now()-source_stamp<=1.:
            raise HardwareFault('fresh finite nonfuture joint source timestamp required')
        if (not all(n in self.joints for n in self.JOINT_NAMES) or
                time.monotonic()-self.joint_rx_monotonic>1. or
                not np.all(np.isfinite(self.current_joint_vector()))):
            raise HardwareFault('complete fresh finite joint state required')

    def event(self,step,result,**details):
        if step=='单苹果现场':
            step,result,details='多果冻结目标','SELECTED',{'target_id':self.target.id}
        super().event(step,result,**details)

    def spin_until(self,predicate,timeout,label):
        end=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            self.guard()
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate(): return True
        return False

    def settle(self,seconds):
        with self.log.phase('gripper_settle',requested_s=seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                self.guard()
                rclpy.spin_once(self,timeout_sec=min(.02,max(0.,end-time.monotonic())))

    def tcp_position(self):
        self.guard()
        transform=self.tf_buffer.lookup_transform(self.base,self.tcp,rclpy.time.Time())
        stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
        if not 0<=self.ros_now()-stamp<=1.:
            raise HardwareFault('stale TCP transform')
        t=transform.transform.translation
        return np.asarray([t.x,t.y,t.z],dtype=float)

    def cartesian_to(self,*args,**kwargs):
        trajectory=super().cartesian_to(*args,**kwargs)
        self.cached.append(trajectory)
        return trajectory

    def check_start(self,trajectory):
        self.guard()
        jt=trajectory.joint_trajectory
        if set(jt.joint_names)!=set(self.JOINT_NAMES) or not jt.points:
            raise RuntimeError('trajectory must include every arm joint')
        current=self.current_joint_vector(jt.joint_names)
        start=np.asarray(jt.points[0].positions)
        if start.shape!=current.shape or not np.all(np.isfinite(start)) or np.max(np.abs(start-current))>.02:
            raise RuntimeError('cached trajectory start differs from fresh actual joint state')

    def scene_fingerprint(self):
        components=PlanningSceneComponents()
        components.components=(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.TRANSFORMS | PlanningSceneComponents.OCTOMAP)
        response=self.get_scene_client.call_async(GetPlanningScene.Request(components=components)).result()
        if response is None: raise HardwareFault('planning scene unavailable')
        self.scene_reference=geometry_signature(response.scene)
        return hashlib.sha256(json.dumps(self.scene_reference,sort_keys=True).encode()).hexdigest()

    def apply_scene(self,apple,radius):
        top=super().apply_scene(apple,radius)
        objects=[]; identities=set()
        for index,d in enumerate(self.snapshot_detections):
            if d['class_id']==self.target.class_id and np.linalg.norm(np.asarray(d['center_xyz_m'])-self.target.center)<.012:
                continue
            center,_=apply_visual_xy_correction(np.asarray(d['center_xyz_m']),self.cfg,
                                                 disabled_by_cli=self.args.disable_visual_xy_correction)
            identity=f'multi_fruit_neighbor_{index}'
            identities.add(identity)
            obj=CollisionObject(); obj.header.frame_id=self.base; obj.id=identity
            obj.operation=CollisionObject.ADD
            obj.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[float(d['radius_m'])])]
            pose=Pose(); pose.orientation.w=1.; pose.position.x,pose.position.y,pose.position.z=map(float,center)
            obj.primitive_poses=[pose]; objects.append(obj)
        for identity in self.neighbor_ids-identities:
            obj=CollisionObject(); obj.id=identity; obj.operation=CollisionObject.REMOVE; objects.append(obj)
        scene=PlanningScene(); scene.is_diff=True; scene.world.collision_objects=objects
        response=self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)).result()
        if response is None or not response.success: raise RuntimeError('neighbor collision geometry rejected')
        self.neighbor_ids=identities
        return top

    def prepare(self,target,budget,record):
        self.transport.budget=budget
        self.target=target; self.cached=[]
        self.planning_detections=deepcopy(self.snapshot_detections)
        self.guard()
        if np.max(np.abs(self.current_joint_vector()-self.scan_vector()))>scan_arrival_tolerance(self.cfg):
            raise RuntimeError('arm must already be at SCAN; this runner never performs initial repositioning')
        raw=np.asarray(target.center)
        corrected,correction=apply_visual_xy_correction(raw,self.cfg,disabled_by_cli=self.args.disable_visual_xy_correction)
        apple=AppleTarget(raw,corrected,target.radius_m,{'confidence':target.confidence,'class_id':target.class_id},correction)
        mode=SimpleNamespace(verify_only=False,auto_return=True,place_target='return')
        prepared=prepare_round(self,self.cfg,self.args,mode,apple,record)
        # Existing prepare_round supplies VERIFY/GRASP/LIFT/RELEASE/RETREAT paths.
        if len(self.cached)!=5: raise RuntimeError('unexpected return preflight path count')
        retreat_state=self.trajectory_end_state(self.cached[-1])
        scan=self.plan(self.make_joint_goal(self.scan_vector()),'PREGRASP -> SCAN preflight',start_state=retreat_state)
        trajectories=[prepared.pre_trajectory]+list(self.cached)+[scan]
        fingerprint=self.scene_fingerprint()
        budget.remaining()
        return SimpleNamespace(round=prepared,trajectories=trajectories,scene=fingerprint,
                               detections=self.planning_detections,selected=deepcopy(target))

    def revalidate(self,prepared,budget):
        self.transport.budget=budget
        self.check_start(prepared.trajectories[0])
        self.current_state_is_valid()
        return (same_detections(prepared.detections,self.snapshot_detections) and
                self.scene_fingerprint()==prepared.scene)

    def execute(self,prepared,record):
        # Runner calls this with a bundle; inherited execution is used for each trajectory below.
        self.transport.budget=None
        self.arming=True
        self.guard()
        self.scene_changed=False
        try:
            self.guard()
            if self.gripper is None:
                self.gripper=GuardedGripper(self)
                with self.log.phase('gripper_configure'):
                    self.gripper.configure(self.cfg['modbus_gripper'])
            self.validate_selected_snapshot(prepared)
            record['motion_started']=True
            with self.log.phase('gripper_open'):
                self.gripper.set_opening_mm(self.cfg['modbus_gripper'],self.cfg['modbus_gripper']['open_mm'])
            p=prepared.round
            poses=[p.pre_pose,p.verify_pose,p.grasp_pose,p.lift_pose,p.grasp_pose,p.pre_pose,None]
            labels=['PREGRASP','VERIFY','GRASP','LIFT','RELEASE','RETREAT','SCAN']
            for trajectory,pose,label in zip(prepared.trajectories,poses,labels):
                if label=='PREGRASP':
                    self.validate_selected_snapshot(prepared)
                    self.active=True
                with self.log.phase('actuation_revalidation',waypoint=label):
                    self.check_start(trajectory)
                    self.current_state_is_valid()
                    if self.scene_fingerprint()!=prepared.scene: raise HardwareFault('scene differs from preflight')
                with self.log.phase('trajectory_execution',waypoint=label):
                    FixedScanVerify.execute(self,trajectory,label)
                with self.log.phase('arrival',waypoint=label):
                    if pose is None: self.wait_scan()
                    else: self.wait_tcp_arrival(pose,label)
                if label=='GRASP':
                    with self.log.phase('gripper_close'):
                        self.gripper.set_opening_mm(self.cfg['modbus_gripper'],self.cfg['modbus_gripper']['close_mm'])
                    record['gripper_close_commanded']=True
                    self.settle(float(self.cfg['modbus_gripper']['close_settle_s']))
                if label=='LIFT': record['lift_completed']=True
                if label=='RELEASE':
                    with self.log.phase('gripper_release'):
                        self.gripper.set_opening_mm(self.cfg['modbus_gripper'],self.cfg['modbus_gripper']['open_mm'])
                    record['gripper_reopened']=True
                    self.settle(1.)
            record['returned_to_scan']=True
        finally:
            self.active=self.arming=False

    def validate_selected_snapshot(self,prepared):
        frozen=prepared.selected
        current=self.manager.tracks.get(frozen.id)
        if (not self.manager.fresh(self.ros_now()) or frozen.id not in self.manager.visible or
                current is None or current.ambiguous or current.stable<self.manager.stable_frames or
                np.linalg.norm(np.asarray(current.center)-frozen.center)>.008 or
                not same_detections(prepared.detections,self.snapshot_detections)):
            raise HardwareFault('target/neighbor snapshot changed or stale before first trajectory')

    def cancel(self):
        self.transport.fault=self.transport.fault or 'session fault; stop unconfirmed'
        self.transport.cancel()


def replay(args):
    now=[10.]
    manager=TargetManager(stable_frames=args.stable_frames)
    class ReplayBackend:
        def prepare(self,target,budget,record):
            budget.remaining(); return target
        def revalidate(self,prepared,budget): return True
        def safe(self): return False
    runner=Runner(manager,ReplayBackend(),ros_now=lambda:now[0])
    fruits=[dict(class_id=49,class_name='orange',confidence=c,center_xyz_m=[x,0.,.1],radius_m=.03)
            for x,c in [(.2,.75),(.4,.95),(.6,.85)]]
    for _ in range(3):
        for _ in range(args.stable_frames):
            now[0]+=.1
            manager.update(dict(version=1,frame='base_link',capture_stamp_s=now[0],valid=True,detections=fruits),now[0])
        runner.step()
    return dict(mode='DETERMINISTIC_SOFTWARE_REPLAY',hardware_connected=False,
                timing_scope='software control only; no perception model, MoveIt, or robot benchmark',
                status=manager.summary(now[0]),rounds=runner.records,phases=runner.log.spans)


def main(argv=None):
    args=parse_args(argv)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.replay:
        result=replay(args)
        args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False)); return 0
    cfg=load_config(args.config)
    manager=TargetManager(class_id=47 if args.target=='apple' else 49,frame=cfg['frames']['base'],
        stable_frames=args.stable_frames,max_age_s=args.max_capture_age_s,
        empty_frames=args.empty_frames,empty_interval_s=args.empty_interval_s,
        min_confidence=float(cfg['vision']['min_confidence']),min_radius_m=float(cfg['vision']['min_radius_m']),
        max_radius_m=float(cfg['vision']['max_radius_m']))
    log=PhaseLog(); rclpy.init(); node=None; runner=None
    status='STARTING'
    try:
        node=MultiFruitNode(cfg,args,manager,log)
        runner=Runner(manager,node,execute=args.execute,operator_confirmed=args.operator_confirmed,
                      planning_budget_s=args.planning_budget_s,ros_now=node.ros_now,log=log)
        # Passive warmup only. No service launch, robot repositioning or gripper initialization.
        deadline=time.monotonic()+float(cfg['safety']['service_timeout_s'])
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if all(n in node.joints for n in node.JOINT_NAMES): break
        previous=None; previous_rounds=0; persisted=time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            status=runner.step()
            changed=status!=previous or len(runner.records)!=previous_rounds
            if status!=previous:
                print(status,flush=True); previous=status
            if changed or time.monotonic()-persisted>=2.:
                write_log(args.output,dict(mode='EXECUTE' if args.execute else 'PLAN_ONLY',
                    status=status,rounds=runner.records,phases=log.spans,events=node.events,
                    ongoing_wait=runner.wait_timer.snapshot()))
                previous_rounds=len(runner.records); persisted=time.monotonic()
            if runner.fault or (args.max_rounds and len(runner.records)>=args.max_rounds): break
        return 2 if runner.fault else 0
    except (KeyboardInterrupt,Exception) as error:
        status='FAULT_LATCHED_STOP_UNCONFIRMED'
        if node is not None: node.cancel()
        print(f'{status}: {error}',flush=True)
        return 2
    finally:
        if runner is not None: runner.wait_timer.close()
        if node is not None:
            if node.transport.fault:
                # Drain late acceptance callbacks so accepted pending goals receive cancellation.
                deadline=time.monotonic()+1.
                while rclpy.ok() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.02)
            args.output.write_text(json.dumps(dict(mode='EXECUTE' if args.execute else 'PLAN_ONLY',status=status,
                rounds=[] if runner is None else runner.records,phases=log.spans,events=node.events,
                ongoing_wait=None if runner is None else runner.wait_timer.snapshot(),
                fault=node.transport.fault,stop_confirmed=False if node.transport.fault else None),indent=2,ensure_ascii=False),encoding='utf-8')
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': raise SystemExit(main())
