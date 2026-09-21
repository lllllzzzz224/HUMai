"""Independent fixed-base RM kinematics. Poses are T_parent_child 4x4 matrices.

The model lives at base identity, retains the named gripper_tcp, and has no
reference to the scene/controller. Newton FK only writes this object's scratch
state. URDF matrix FK provides independent final acceptance verification.
"""
from dataclasses import dataclass
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import newton
import newton.ik as ik
import warp as wp
from . import rm65
from . import rm65_control as rc


def validate_pose(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4,4) or not np.isfinite(pose).all():
        raise ValueError('pose must be a finite 4x4 matrix')
    r = pose[:3,:3]
    if not np.allclose(pose[3], [0,0,0,1], atol=1e-8, rtol=0) or not np.allclose(r.T@r,np.eye(3),atol=1e-7,rtol=0) or abs(np.linalg.det(r)-1)>1e-7:
        raise ValueError('pose must have a proper orthonormal rotation and homogeneous last row')
    return pose.copy()


def pose_from_xyz_quat(xyz, quat_xyzw):
    xyz, quat = rc._vector(xyz,3), rc._vector(quat_xyzw,4)
    if abs(np.linalg.norm(quat)-1)>1e-6:
        raise ValueError('quaternion must be unit length (xyzw)')
    result=np.eye(4); result[:3,3]=xyz; result[:3,:3]=Rotation.from_quat(quat).as_matrix()
    return result


def target_pose_base_to_world(target_pose_base, T_world_base):
    return validate_pose(T_world_base) @ validate_pose(target_pose_base)


def target_pose_world_to_base(target_pose_world, T_world_base):
    return np.linalg.inv(validate_pose(T_world_base)) @ validate_pose(target_pose_world)


def pose_error(actual, target):
    actual, target=validate_pose(actual),validate_pose(target)
    return (float(np.linalg.norm(actual[:3,3]-target[:3,3])),
            float(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude()))


@dataclass
class IKResult:
    status: str
    q_arm: np.ndarray | None
    candidates: list
    message: str = ''


class RM65Kinematics:
    def __init__(self, device='cuda:0'):
        self.limits=rc.description()
        self.soft_lower=self.limits['lower'][:6]+rc.ARM_COMMAND_MARGIN
        self.soft_upper=self.limits['upper'][:6]-rc.ARM_COMMAND_MARGIN
        builder=newton.ModelBuilder()
        builder.add_urdf(str(rm65.URDF_PATH), xform=wp.transform_identity(), floating=False,
                         enable_self_collisions=False)
        self.model=builder.finalize(device=device)
        self.state=self.model.state()
        js=rc.resolve_names(self.model.joint_label)
        self.qs=self.model.joint_q_start.numpy()[js]
        self.ds=self.model.joint_qd_start.numpy()[js]
        self.tcp=rm65._unique_named_index(self.model.body_label,'gripper_tcp')
        self._qd=wp.zeros(self.model.joint_dof_count,dtype=wp.float32,device=self.model.device)
        joints={j.find('child').get('link'): j for j in ET.parse(rm65.URDF_PATH).getroot().findall('joint')}
        chain=[]; link='gripper_tcp'
        while link!='base_link':
            j=joints[link]; origin=j.find('origin')
            matrix=np.eye(4)
            matrix[:3,3]=rm65._vector(origin,'xyz')
            matrix[:3,:3]=Rotation.from_euler('xyz',rm65._vector(origin,'rpy')).as_matrix()
            axis=j.find('axis')
            axis=np.array([1,0,0] if axis is None else [float(x) for x in axis.get('xyz').split()])
            chain.append((j.get('name'),j.get('type'),matrix,axis))
            link=j.find('parent').get('link')
        self._chain=chain[::-1]

    def fk(self,q_arm,q_fingers=(0.,0.)):
        """Independent URDF matrix-chain T_base_tcp; no limit projection."""
        q=rc._vector(q_arm,6); rc._vector(q_fingers,2)
        positions=dict(zip(rm65.ARM_JOINT_NAMES,q)); result=np.eye(4)
        for name,kind,origin,axis in self._chain:
            result=result@origin
            if kind=='revolute':
                motion=np.eye(4); motion[:3,:3]=Rotation.from_rotvec(axis*positions[name]).as_matrix()
                result=result@motion
        return result

    def fk_newton(self,q_arm,q_fingers=(0.,0.)):
        q=self.model.joint_q.numpy().copy()
        q[self.qs]=np.r_[rc._vector(q_arm,6),rc._vector(q_fingers,2)]
        newton.eval_fk(self.model,wp.array(q,dtype=wp.float32,device=self.model.device),self._qd,self.state)
        pose=self.state.body_q.numpy()[self.tcp]
        # Float32 FK quaternions can accumulate tiny normalization error.
        quat=pose[3:].astype(float); quat/=np.linalg.norm(quat)
        return pose_from_xyz_quat(pose[:3],quat)

    def jacobian(self,q_arm,epsilon=1e-6):
        q=rc._vector(q_arm,6); jac=np.empty((6,6))
        for i in range(6):
            d=np.zeros(6); d[i]=epsilon
            a,b=self.fk(q-d),self.fk(q+d)
            jac[:3,i]=(b[:3,3]-a[:3,3])/(2*epsilon)
            jac[3:,i]=Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).as_rotvec()/(2*epsilon)
        return jac

    def singular_values(self,q_arm):
        return np.linalg.svd(self.jacobian(q_arm),compute_uv=False)

    def solve(self,target_pose_base,current_q,q_fingers=(0.,0.),extra_seeds=(),
              max_iterations=100,timeout_s=30.,collision_check=None,continuity_reference=None):
        """Measured q is first seed; choose closest valid result in actual radians.

        continuity_reference optionally supplies the previous waypoint seed after
        the measured seed. No angle wrapping, target writes, or limit clipping.
        collision_check receives an arm-q copy; False or an exception rejects it.
        Timeout includes solver construction and compilation and fails closed.
        """
        start=time.monotonic()
        candidates=[]
        try:
            target=validate_pose(target_pose_base); current=rc._vector(current_q,6)
            fingers=rc._vector(q_fingers,2)
            if np.any(fingers<self.limits['lower'][6:]) or np.any(fingers>self.limits['upper'][6:]):
                raise ValueError('finger request exceeds source bounds')
            if not isinstance(max_iterations,(int,np.integer)) or max_iterations<0 or not np.isfinite(timeout_s) or timeout_s<=0:
                raise ValueError('invalid iterations or timeout')
            seeds=[current.copy()]
            if continuity_reference is not None: seeds.append(rc._vector(continuity_reference,6))
            seeds.extend(rc._vector(seed,6) for seed in extra_seeds)
        except (ValueError,TypeError) as exc:
            return IKResult('invalid_input',None,candidates,str(exc))
        device=self.model.device
        candidates=[dict(seed=seed.copy(),q_arm=None,distance=None,position_error=None,
                         rotation_error=None,accepted=False,reason='not_evaluated') for seed in seeds]
        record=candidates[0]
        def expired():
            return time.monotonic()-start>=timeout_s
        def timed_out():
            record['reason']='timeout'; record['accepted']=False
            for pending in candidates:
                if pending['reason']=='not_evaluated': pending['reason']='timeout'
            return IKResult('timeout',None,candidates)
        try:
            if expired(): return timed_out()
            lower=self.model.joint_limit_lower.numpy().copy(); upper=self.model.joint_limit_upper.numpy().copy()
            lower[self.ds[:6]]=self.soft_lower; upper[self.ds[:6]]=self.soft_upper
            objectives=[ik.IKObjectivePosition(self.tcp,wp.vec3(0,0,0),wp.array([target[:3,3]],dtype=wp.vec3,device=device)),
                        ik.IKObjectiveRotation(self.tcp,wp.quat_identity(),wp.array([Rotation.from_matrix(target[:3,:3]).as_quat()],dtype=wp.vec4,device=device)),
                        ik.IKObjectiveJointLimit(wp.array(lower,dtype=wp.float32,device=device),wp.array(upper,dtype=wp.float32,device=device),weight=1.)]
            if expired(): return timed_out()
            solver=ik.IKSolver(self.model,1,objectives,optimizer='lm',jacobian_mode='analytic')
            for record in candidates:
                seed=record['seed'];record['reason']=''
                if expired(): return timed_out()
                q=self.model.joint_q.numpy().copy(); q[self.qs]=np.r_[seed,fingers]
                buffer=wp.array(q.reshape(1,-1),dtype=wp.float32,device=device)
                # Small chunks allow wall-clock checks during real GPU work.
                for offset in range(0,max_iterations,10):
                    solver.step(buffer,buffer,iterations=min(10,max_iterations-offset))
                    wp.synchronize_device(device)
                    if expired(): return timed_out()
                solved=buffer.numpy()[0]; arm=solved[self.qs[:6]].astype(float)
                record['q_arm']=arm.copy()
                if not np.isfinite(solved).all(): record['reason']='nonfinite'
                else:
                    record['distance']=float(np.linalg.norm(arm-current))
                    p,r=pose_error(self.fk(arm),target)
                    record.update(position_error=p,rotation_error=r)
                    if np.any(arm<self.soft_lower) or np.any(arm>self.soft_upper): record['reason']='soft_limit'
                    elif not np.allclose(solved[self.qs[6:]],fingers,atol=1e-7,rtol=0): record['reason']='finger_changed'
                    elif p>.0005 or r>np.deg2rad(.1): record['reason']='pose_residual'
                    elif collision_check is not None:
                        try:
                            verdict=collision_check(arm.copy())
                            if verdict is not None and not bool(verdict): record['reason']='collision'
                        except Exception as exc:
                            if expired(): return timed_out()
                            if hasattr(exc,'pair') and hasattr(exc,'distance'):
                                record['reason']='collision: '+str(exc)
                            else:
                                record['reason']='collision_check_error: '+str(exc)
                                return IKResult('collision_check_error',None,candidates,str(exc))
                if expired(): return timed_out()
                record['accepted']=not bool(record['reason'])
            if expired(): return timed_out()
            valid=[c for c in candidates if c['accepted']]
            if valid:
                return IKResult('success',min(valid,key=lambda c:c['distance'])['q_arm'].copy(),candidates)
            return IKResult('no_solution',None,candidates)
        except Exception as exc:
            if expired(): return timed_out()
            record['reason']='solver_error: '+str(exc);record['accepted']=False
            return IKResult('solver_error',None,candidates,str(exc))
