"""RM65 Stage 3.2 native POSITION actuator configuration and frame targets.

No state writes, kinematics solver, camera, robot driver, or harvesting imports.
MuJoCo does not enforce joint_velocity_limit: RATE limits the target only;
check_state independently rejects excessive measured joint velocity.
"""
import xml.etree.ElementTree as ET
import newton
import numpy as np
from . import rm65

NAMES = rm65.ARM_JOINT_NAMES + rm65.GRIPPER_MOVABLE_JOINT_NAMES
HOME = np.zeros(8)
KP = np.array([400, 600, 400, 120, 100, 60, 1000, 1000.], dtype=float)
KD = np.array([12, 20, 12, 4, 3, 2, 12, 12.], dtype=float)
# Conservative running limits: half the arm source rating, 2 N per unloaded
# finger. Finger P force for a 0.02/60 m target increment is only 0.333 N.
# Closed-finger stall is capped at 2 N, never at the source's 100 N.
EFFORT = np.array([30, 30, 15, 5, 5, 5, 2, 2.], dtype=float)
RATE = np.array([.10]*6 + [.02]*2)
# Command-only arm margin; never applied to compiled limits or gripper DOFs.
# Qualified only at arm RATE=.10, existing KP/KD, 60 Hz and 3 substeps.
# User-selected .01 rad was based on .000518 rad small-move overshoot.
# Soft-boundary revalidation measured .000615017 rad (~16.26x clearance budget).
# Changed control conditions require revalidation; no global safety claim.
ARM_COMMAND_MARGIN = .01


def resolve_names(labels, names=NAMES):
    result = []
    for name in names:
        hits = [i for i, label in enumerate(labels) if str(label).rsplit('/', 1)[-1] == name]
        if len(hits) != 1:
            raise ValueError(f"joint {name}: expected exactly one, found {len(hits)}")
        result.append(hits[0])
    return np.asarray(result, dtype=int)


def description():
    rm65.inspect_urdf()
    root = ET.parse(rm65.URDF_PATH).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    data = {k: [] for k in ('lower', 'upper', 'velocity', 'effort', 'axis')}
    for name in NAMES:
        j = joints[name]
        for key in ('lower', 'upper', 'velocity', 'effort'):
            data[key].append(float(j.find('limit').get(key)))
        data['axis'].append([float(v) for v in j.find('axis').get('xyz').split()])
    data = {k: np.asarray(v, dtype=float) for k, v in data.items()}
    if not all(np.isfinite(v).all() for v in data.values()):
        raise ValueError('nonfinite URDF control metadata')
    return data


def configure(builder, kp=KP, kd=KD):
    """Configure only named RM DOFs, before finalize/solver construction."""
    limits = description()
    kp, kd = _vector(kp, 8), _vector(kd, 8)
    if np.any(kp <= 0) or np.any(kd < 0) or np.any(EFFORT > limits['effort']):
        raise ValueError('invalid gain or running effort limit')
    for k, j in enumerate(resolve_names(builder.joint_label)):
        ds = builder.joint_qd_start[j]
        builder.joint_target_mode[ds] = newton.JointTargetMode.POSITION
        builder.joint_target_ke[ds] = float(kp[k])
        builder.joint_target_kd[ds] = float(kd[k])
        builder.joint_effort_limit[ds] = float(EFFORT[k])
        builder.joint_velocity_limit[ds] = float(limits['velocity'][k])


def _vector(value, length):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f'expected {length} finite positions')
    return result


class JointController:
    """Single-RM controller. update(frame) is called exactly once at 60 Hz.

    All physics substeps share the resulting control buffer. Reusing a frame
    number is an error rather than silently applying another slew increment.
    """
    def __init__(self, model, control):
        self.model, self.control = model, control
        self.limits = description()
        self.soft_lower = self.limits['lower'][:6] + ARM_COMMAND_MARGIN
        self.soft_upper = self.limits['upper'][:6] - ARM_COMMAND_MARGIN
        self.js = resolve_names(model.joint_label)
        self.qs = model.joint_q_start.numpy()[self.js]
        self.ds = model.joint_qd_start.numpy()[self.js]
        self.ts = model.joint_target_q_start.numpy()[self.js]
        self.goal = _vector(model.joint_q.numpy()[self.qs], 8).copy()
        if np.any(self.goal < self.limits['lower']-1e-6) or np.any(self.goal > self.limits['upper']+1e-6):
            raise ValueError('initial position exceeds URDF limits')
        self.sent = np.clip(self.goal, self.limits['lower'], self.limits['upper'])
        self.raw_arm_request = self.goal[:6].copy()
        self.effective_arm_target = self.goal[:6].copy()
        self.soft_clamped = np.zeros(6, dtype=bool)
        self.last_frame = -1
        self._write()

    def _write(self):
        targets = self.control.joint_target_q.numpy()
        targets[self.ts] = self.sent
        self.control.joint_target_q.assign(targets)

    def set_arm_targets(self, values):
        values = _vector(values, 6)
        clamped = np.clip(values, self.soft_lower, self.soft_upper)
        self.raw_arm_request = values.copy()
        self.effective_arm_target = clamped.copy()
        self.soft_clamped = values != clamped
        self.goal[:6] = clamped
        return bool(np.any(self.soft_clamped))

    def set_gripper_open_fraction(self, value):
        value = _vector([value], 1)[0]
        fraction = np.clip(value, 0., 1.)
        self.goal[6:] = .06 * (1. - fraction)
        return bool(value != fraction)

    def update(self, frame):
        if not isinstance(frame, (int, np.integer)) or frame != self.last_frame + 1:
            raise ValueError('target update requires the next unique 60 Hz control frame')
        self.sent += np.clip(self.goal-self.sent, -RATE/60., RATE/60.)
        self.sent = np.clip(self.sent, self.limits['lower'], self.limits['upper'])
        self._write()
        self.last_frame = frame

    def read_arm_state(self, state):
        return state.joint_q.numpy()[self.qs[:6]], state.joint_qd.numpy()[self.ds[:6]]

    def read_gripper_state(self, state):
        return state.joint_q.numpy()[self.qs[6:]], state.joint_qd.numpy()[self.ds[6:]]

    def hold_current(self, state):
        self.check_state(state)
        current = state.joint_q.numpy()[self.qs]
        self.set_arm_targets(current[:6])
        self.goal[6:] = np.clip(current[6:], self.limits['lower'][6:], self.limits['upper'][6:])

    def is_at_target(self, state, arm_tolerance=.001, finger_tolerance=.0005):
        q, v = state.joint_q.numpy()[self.qs], state.joint_qd.numpy()[self.ds]
        return bool(np.all(np.abs(q-self.goal) <= [arm_tolerance]*6+[finger_tolerance]*2)
                    and np.all(np.abs(v) <= [.005]*6+[.001]*2))

    def check_state(self, state):
        arrays = (state.joint_q.numpy(), state.joint_qd.numpy(), state.body_q.numpy(), state.body_qd.numpy())
        if not all(np.isfinite(a).all() for a in arrays):
            raise RuntimeError('STOP: NaN/Inf in state')
        q, v = arrays[0][self.qs], arrays[1][self.ds]
        if np.any(q < self.limits['lower']-1e-4) or np.any(q > self.limits['upper']+1e-4):
            raise RuntimeError(f'STOP: joint limit violation: {q}')
        if np.any(np.abs(v) > np.minimum(self.limits['velocity'], [.3]*6+[.06]*2)):
            raise RuntimeError(f'STOP: abnormal joint velocity: {v}')
        return q, v
