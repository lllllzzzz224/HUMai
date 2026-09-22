#!/usr/bin/env python3
"""Bounded isolated-domain synthetic integration test. Never executes motion."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np
import rclpy
from rclpy.action import get_action_client_names_and_types_by_node, get_action_server_names_and_types_by_node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPositionFK, GetStateValidity

ROOT = Path('/home/li/桌面/9月机械臂实验（算法）')
PRODUCT = ROOT / 'development/hand_eye_calibration/apple_pick_v2'
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(PRODUCT))
from obstacle_scene_ros import (RosPlanTransport, JOINTS, ensure_isolated, raw_message,
    scene_digest, scene_components, recorded_state, make_plan_goal, verify_trajectory, box_object)

report = {'kind': 'ARTIFICIAL_SYNTHETIC_INTEGRATION_ONLY', 'validated_pregrasp': False,
          'executable': False, 'attempts': [], 'started_unix': time.time()}
own_ids = set()
baseline = None
deadline = time.monotonic() + 500

def save(name, data):
    (OUT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2))

def checkpoint():
    save('report.json', report)
    if time.monotonic() > deadline:
        raise TimeoutError('bounded_synthetic_validation_deadline')

def mutate(transport, objects):
    diff = PlanningScene(); diff.is_diff = True; diff.robot_state.is_diff = True
    diff.world.collision_objects = objects
    if not transport.apply(diff):
        raise RuntimeError('synthetic_scene_apply_rejected')

def clear_owned(transport):
    existing = {o.id for o in transport.read().world.collision_objects}
    removes=[]
    for ident in sorted(own_ids & existing):
        obj=CollisionObject(id=ident, operation=CollisionObject.REMOVE)
        obj.header.frame_id='base_link'; obj.pose.orientation.w=1.
        removes.append(obj)
    if removes: mutate(transport, removes)

def validity(transport, state):
    response = transport.call(transport.valid_client, GetStateValidity.Request(robot_state=state, group_name='rm_group'))
    return {'valid': response.valid, 'contacts': raw_message(response)['contacts']}

def fk(transport, state):
    cli=transport.node.create_client(GetPositionFK, '/compute_fk')
    try:
        req=GetPositionFK.Request(); req.header.frame_id='base_link'
        req.fk_link_names=['Link4','Link5','Link6','gripper_body_approx','tcp_link']
        req.robot_state=state
        res=transport.call(cli, req)
        if res.error_code.val != 1: raise RuntimeError('fk_failed')
        return {n:[p.pose.position.x,p.pose.position.y,p.pose.position.z] for n,p in zip(res.fk_link_names,res.pose_stamped)}
    finally: transport.node.destroy_client(cli)

def plan(transport, scene, start, goal, label):
    entry={'label':label,'q_start':list(start),'q_goal':list(goal),'goal_source':'artificial_joint_perturbation'}
    request=make_plan_goal(scene,start,goal)
    save(label+'.goal.json',raw_message(request))
    began=time.monotonic()
    original_wait=transport.wait
    def audited_wait(future, timeout=20.):
        result=original_wait(future,timeout)
        if hasattr(result,'status') and hasattr(result,'result'):
            save(label+'.action_result.json',{'status':result.status,'result':raw_message(result.result)})
            entry['action_status']=result.status
            entry['moveit_error_code']=result.result.error_code.val
        return result
    transport.wait=audited_wait
    try:
        result=transport.plan(request)
        entry['planning_time_s']=result.planning_time
        entry['moveit_error_code']=result.error_code.val
        if result.error_code.val != 1:
            entry['status']='MOVEIT_REJECTED'; return entry,None
        samples=[]
        def checked(state):
            val=validity(transport,state)
            samples.append({'q':list(state.joint_state.position),**val})
            return val['valid']
        verified=verify_trajectory(result.planned_trajectory,result.trajectory_start,checked,step_rad=.01)
        entry['status']='PLAN_ONLY_INTERPOLATION_VALID'
        entry['samples']=len(verified); entry['trajectory_points']=len(result.planned_trajectory.joint_trajectory.points)
        entry['max_sample_joint_step_rad']=float(np.max(np.abs(np.diff(verified,axis=0)))) if len(verified)>1 else 0.
        save(label+'.samples.json',samples)
        entry['start_error_rad']=float(np.max(np.abs(verified[0]-np.asarray(start))))
        entry['goal_error_rad']=float(np.max(np.abs(verified[-1]-np.asarray(goal))))
        return entry,result
    except Exception as exc:
        entry['status']='REJECTED_OR_VERIFICATION_FAILED';entry['error']=str(exc)
        return entry,None
    finally:
        transport.wait=original_wait
        entry['wall_time_s']=time.monotonic()-began
        report['attempts'].append(entry);checkpoint()

def graph_audit(node):
    nodes=[]
    for name, ns in node.get_node_names_and_namespaces():
        item={'name':name,'namespace':ns}
        item['action_servers']=get_action_server_names_and_types_by_node(node,name,ns)
        item['action_clients']=get_action_client_names_and_types_by_node(node,name,ns)
        nodes.append(item)
    return {'nodes':nodes,'services':node.get_service_names_and_types(),
            'topics':node.get_topic_names_and_types()}

rclpy.init();node=rclpy.create_node('synthetic_plan_only_integration_audit')
transport=RosPlanTransport(node)
try:
    ensure_isolated(transport)
    report['domain']=os.environ.get('ROS_DOMAIN_ID')
    report['discovery_range']=os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
    report['allow_trajectory_execution']=False
    report['imported_product_sha256']=hashlib.sha256((PRODUCT/'obstacle_scene_ros.py').read_bytes()).hexdigest()
    baseline=transport.read()
    report['baseline_scene_digest']=scene_digest(baseline)
    report['baseline_world_count']=len(baseline.world.collision_objects)
    report['baseline_attached_ids']=[a.object.id for a in baseline.robot_state.attached_collision_objects]
    if any(o.id.startswith('auto_obstacle_synthetic_') for o in baseline.world.collision_objects):
        raise RuntimeError('synthetic_namespace_already_occupied')
    save('baseline_scene.json',raw_message(baseline));save('graph_before.json',graph_audit(node))
    snapshot=json.loads((OUT.parent/'registered_replay/scene.json').read_text())
    q0=np.asarray(snapshot['observation']['joint_positions_rad'],float)
    report['q_start_source']='recorded_controller_bracket_replayed_as_synthetic_test_origin'
    report['q_start']=q0.tolist()
    report['start_state_check']=validity(transport,recorded_state(baseline,q0))
    if not report['start_state_check']['valid']:raise RuntimeError('recorded_origin_invalid')
    checkpoint()
    detour=None
    for candidate_idx,(joint,delta) in enumerate([(0,-.6),(0,.5),(1,.35),(1,-.35),(2,.4),(2,-.4)]):
        checkpoint();clear_owned(transport)
        q1=q0.copy();q1[joint]+=delta
        goal_val=validity(transport,recorded_state(baseline,q1))
        report.setdefault('candidate_goals',[]).append({'joint':joint+1,'delta':delta,'q':q1.tolist(),**goal_val})
        if not goal_val['valid']:continue
        label=f'candidate_{candidate_idx}_baseline'
        baseline_result,base_plan=plan(transport,baseline,q0,q1,label)
        if base_plan is None:continue
        midpoint=(q0+q1)/2
        locations=fk(transport,recorded_state(baseline,midpoint))
        for link in ['tcp_link','Link6','Link5']:
            for size in [.025,.045,.07]:
                checkpoint();clear_owned(transport)
                ident='auto_obstacle_synthetic_detour'
                own_ids.add(ident)
                box=box_object(ident,locations[link],[size]*3)
                mutate(transport,[box]);scene=transport.read()
                checks={name:validity(transport,recorded_state(scene,q)) for name,q in [('start',q0),('midpoint',midpoint),('goal',q1)]}
                case={'candidate':candidate_idx,'link':link,'size_m':size,'center_m':locations[link],'checks':checks}
                report.setdefault('obstacle_trials',[]).append(case);checkpoint()
                if not (checks['start']['valid'] and checks['goal']['valid'] and not checks['midpoint']['valid']):continue
                case['direct_path_samples']=[{'fraction':float(t),**validity(transport,recorded_state(scene,q0+(q1-q0)*t))} for t in np.linspace(0,1,21)]
                dlabel=f'candidate_{candidate_idx}_detour_{link}_{size}'
                attempt,res=plan(transport,scene,q0,q1,dlabel)
                if res is not None:
                    detour={'scene':scene,'q_goal':q1,'label':dlabel,'case':case,'result':res}
                    save('detour_scene.json',raw_message(scene))
                    report['detour']=dict(label=dlabel,**case)
                    # Show only the verified artificial result; never execute.
                    transport.display(res)
                    break
            if detour:break
        if detour:break
    if detour is None:
        report['status']='NO_VALID_DETOUR_FOUND_WITHIN_BOUNDED_SEARCH'
    else:
        clear_owned(transport)
        ident='auto_obstacle_synthetic_full_block';own_ids.add(ident)
        mutate(transport,[box_object(ident,[0.,0.,.4],[2.,2.,2.])])
        blocked=transport.read();save('blocked_scene.json',raw_message(blocked))
        report['full_block_state_checks']={name:validity(transport,recorded_state(blocked,q)) for name,q in [('start',q0),('goal',detour['q_goal'])]}
        attempt,result=plan(transport,blocked,q0,detour['q_goal'],'full_block')
        report['full_block_plan_rejected']=result is None
        report['status']='SYNTHETIC_DETOUR_VALID_AND_FULL_BLOCK_REJECTED' if result is None else 'UNEXPECTED_BLOCKED_PLAN_SUCCESS'
        checkpoint()
except Exception as exc:
    report['status']='SYNTHETIC_TEST_ERROR';report['error']=str(exc);report['traceback']=traceback.format_exc()
finally:
    try:
        clear_owned(transport)
        restored=transport.read();save('restored_scene.json',raw_message(restored))
        report['restored_scene_digest']=scene_digest(restored)
        report['restored_matches_baseline']=baseline is not None and scene_digest(restored)==scene_digest(baseline)
        report['remaining_synthetic_ids']=[o.id for o in restored.world.collision_objects if o.id in own_ids]
        save('graph_after.json',graph_audit(node))
    except Exception as exc:
        report['cleanup_error']=str(exc)
    report['ended_unix']=time.time();report['wall_time_s']=report['ended_unix']-report['started_unix']
    report['owned_ids']=sorted(own_ids);save('report.json',report)
    node.destroy_node();rclpy.shutdown()
print(json.dumps(report,ensure_ascii=False))
