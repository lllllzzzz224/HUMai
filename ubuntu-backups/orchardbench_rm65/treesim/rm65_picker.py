"""Fixed RM adapter for the author's AutoPicker, not a new harvest policy.

REACH/GRASP/PULL are inherited unchanged. GRASP deliberately retains the author's
simulator-truth fruit servo and integral compensation: this is a truth-assisted
baseline with spring-assisted hold, NOT a pure-vision or pure-friction policy.
The fixed installation has no mobile survey, bucket, transport or repeated pick.
"""
import copy
import math
from types import SimpleNamespace
import numpy as np
import warp as wp
from .picker import AutoPicker
from .rm65_kinematics import RM65Kinematics, pose_from_xyz_quat
from .rm65_runtime import ScratchScene, emit
from .rm65_acceptance import EnvironmentClearance
from .rm65_control import RATE


def configure_rm_hold(apples):
    # Anchor is the named gripper_tcp itself. Spring/damping/break physics stays original.
    apples.hold_off = wp.vec3(0., 0., 0.)


class _RMArmIK:
    """Original ArmIK call shape, implemented with the existing RM kinematics."""
    _arm_q = tuple(range(6))

    def __init__(self, owner):
        self.owner = owner
        self.kin = RM65Kinematics(device=owner.runtime.sim.model.device)

    def solve(self, target_base, approach_base, q_init, iters=32):
        p = self.owner
        z = np.asarray(approach_base, float)
        z /= np.linalg.norm(z)
        y = np.cross(z, [0., 0., 1.])
        if np.linalg.norm(y) < 1e-6:
            y = np.array([0., 1., 0.])
        y /= np.linalg.norm(y)
        target = np.eye(4)
        target[:3, :3] = np.column_stack((np.cross(y, z), y, z))
        target[:3, 3] = target_base
        current, _ = p.runtime.controller.read_arm_state(p.sim.state_0)
        p.refresh_scratch()
        result = self.kin.solve(target, current, collision_check=p.check_scratch)
        emit(event='rm_baseline_ik', state=p.state, status=result.status,
             candidates=result.candidates)
        if result.status != 'success':
            p._fail('IK_FAILED: '+result.status)
        return np.r_[result.q_arm, p.runtime.controller.goal[6:]], min(
            c['position_error'] for c in result.candidates if c['accepted'])


class RMBaselinePicker(AutoPicker):
    FINGER_OPEN, FINGER_CLOSED = 0., .06

    def __init__(self, runtime, camera):
        self.runtime = runtime
        tree, sim = runtime.tree, runtime.sim
        names = tree.robot_data['body_indices']
        self.tcp = int(names['gripper_tcp'][0])
        self.base = int(names['base_link'][0])
        # The original initializer consumes generic numeric adapter metadata.
        # This shallow facade never changes the actual TreeModel or its mapping.
        facade = copy.copy(tree)
        facade.robot_data = dict(tree.robot_data, chassis=[self.base], wrist=[self.tcp],
            arm_tq=[runtime.controller.ts], arm_dofs=[runtime.controller.ds],
            arm_home=np.r_[tree.config.robot.rm_ready, 0., 0.])
        super().__init__(sim, facade, camera, SimpleNamespace(), tree.config.robot,
                         ik=_RMArmIK(self))
        indices = [i for i, b in enumerate(tree.apple_bodies)
                   if sim.model.body_label[int(b)].rsplit('/', 1)[-1] == 'apple35']
        if len(indices) != 1:
            raise ValueError('RM baseline requires one named apple35')
        self.target_index = indices[0]
        self.target_body = int(tree.apple_bodies[self.target_index])
        parent = int(tree.apple_data['parent_body'][self.target_index])
        if sim.model.body_label[parent].rsplit('/', 1)[-1] != 'seg294':
            raise ValueError('RM baseline fixture parent must be seg294')
        self._apple_idx = np.array([self.target_body])
        self._apple_glob = np.array([self.target_index])
        self._last_sample_id = -1
        self._phase_start = self._frame
        self._ready_count = 0
        self._verify_count = 0
        self.scratch = ScratchScene(runtime)
        self.clearance = EnvironmentClearance(runtime)
        runtime.allowed_pick_index = self.target_index
        configure_rm_hold(self.apples)
        self.state = 'READY'
        self._set_arm(self.arm_home)
        emit(event='rm_baseline_adapter', truth_assisted=True, spring_assisted_hold=True,
             reused_methods=['AutoPicker._st_reach', 'AutoPicker._st_grasp', 'AutoPicker._st_pull'],
             fixed_base=True, target='apple35', parent='seg294', tcp_body=self.tcp)

    def refresh_scratch(self):
        self.scratch.state.assign(self.sim.state_0)
        self.scratch.snapshot_q = self.sim.state_0.joint_q.numpy().copy()

    def check_scratch(self, q):
        self.scratch.check(q)
        self.clearance.check(self.scratch.state, 'rm_baseline/'+self.state)
        return True

    def _tcp_world(self):
        return self._bq()[self.tcp, :3].copy()

    def _set_arm(self, q_goal):
        from scripts.validate_rm65_camera import scratch_slew_path
        goal = np.asarray(q_goal, float)
        if goal.shape != (8,) or not np.isfinite(goal).all():
            self._fail('INVALID_JOINT_TARGET')
        current, _ = self.runtime.controller.read_arm_state(self.sim.state_0)
        self.refresh_scratch()
        for q in scratch_slew_path(current, goal[:6])[0]:
            self.check_scratch(q)
        # Atomic: no new live target is issued until all scratch samples pass.
        self.runtime.controller.set_arm_targets(goal[:6])
        self._q_goal = goal.copy()

    def _slew_arm(self, rate=None):
        # RMRuntime alone updates the physical target buffer once per frame.
        self._q_cmd = self.runtime.controller.sent.copy()

    def _fingers(self, opening):
        self.runtime.controller.set_gripper_open_fraction(1.-opening/.06)
        self._q_goal[-2:] = opening
        self._q_cmd[-2:] = opening

    def _drive(self, v, w):
        if v or w:
            self._fail('FIXED_BASE_DRIVE_REQUEST')

    def _update_tracks(self):
        sample = self.cam.last_sample
        if sample is None or sample.sample_id == self._last_sample_id:
            return
        self._last_sample_id = sample.sample_id
        self.cam.dets_env = [sample.detections_world]
        super()._update_tracks()

    def _confirmed_targets(self):
        # Explicit truth-assisted single-fruit fixture restriction, not detector input.
        truth = self._bq()[self.target_body, :3]
        return [p for p in super()._confirmed_targets() if np.linalg.norm(p-truth) < .08]

    def _st_scan(self):
        self._fingers(self.FINGER_OPEN)
        targets = self._confirmed_targets()
        if targets:
            base, _ = self._chassis_pose()
            self._target = min(targets, key=lambda p: np.linalg.norm(p[:2]-base[:2]))
            self._goto('REACH')  # Fixed fixture replaces mobile ALIGN; no base commands.
        elif self._t_state > 600:
            self._fail('NO_CONFIRMED_TARGET')

    def _goto(self, state):
        if state == 'TRANSPORT':
            state = 'DETACH_VERIFY'  # No Ridgeback bucket or drop in this fixed variant.
        super()._goto(state)
        self._phase_start = self._frame
        if state in ('GRASP', 'PULL', 'DETACH_VERIFY'):
            name = self.sim.model.body_label[self.target_body].rsplit('/', 1)[-1]
            self.runtime.audit.allowed_touch_pairs = {
                frozenset((name, link)) for link in
                ('gripper_left_finger', 'gripper_right_finger', 'gripper_palm')}
        emit(event='rm_baseline_state', state=state, frame=self._frame)

    def _fail(self, reason):
        self.done = True
        self.runtime.stopped = True
        emit(event='rm_baseline_stop', state=self.state, reason=reason, frame=self._frame)
        raise RuntimeError(reason)

    def _slewing(self):
        return not np.allclose(self.runtime.controller.sent[:6],
                               self.runtime.controller.goal[:6], atol=2e-6, rtol=0)

    def _stalled(self, dist, tol=.005):
        # The author assumes much faster Franka slew. Wait for RM's unchanged
        # target-rate limiter before charging the original feedback watchdog.
        return False if self._slewing() else super()._stalled(dist, tol)

    def update(self):
        if self.done:
            return
        self._frame += 1
        if self._frame-self._phase_start > 7200:
            self._fail('RM_BASELINE_PHASE_TIMEOUT')
        if self.state == 'READY':
            _, v = self.runtime.controller.read_arm_state(self.sim.state_0)
            self._ready_count = self._ready_count+1 if not self._slewing() and np.max(np.abs(v)) <= .005 else 0
            if self._ready_count >= 15:
                self._goto('SCAN')
            return
        self._update_tracks()
        if self.state == 'DETACH_VERIFY':
            if not self.apples.detached[self.target_index] or not self.apples._held_host[self.target_index]:
                self._fail('DETACH_NOT_CONFIRMED')
            if np.linalg.norm(self._bq()[self.target_body, :3]-self._tcp_world()) > .10:
                self._fail('DETACHED_FRUIT_NOT_FOLLOWING_TCP')
            self._verify_count += 1
            if self._verify_count >= 15:
                self.state = 'DONE'; self.done = True
                emit(event='rm_baseline_done', truth_assisted=True, spring_assisted_hold=True)
            return
        # Keep the author's feedback timers, excluding rate-limited transit.
        if self._t_state < 2 or not self._slewing():
            self._t_state += 1
        getattr(self, '_st_'+self.state.lower())()
        self._slew_arm()
