"""Ordered Stage 3.2 validation. Run A tests before invoking B.

No runtime q writes. eval_fk below is only Newton initial-state construction.
All subsequent motion comes from solver.step with POSITION actuators.
"""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import newton
import numpy as np
import warp as wp
from treesim import rm65, rm65_control as rc


def emit(**data):
    print(json.dumps(data, default=lambda a: a.tolist() if isinstance(a, np.ndarray) else float(a)), flush=True)


class Fixture:
    def __init__(self, gravity=0., kp=rc.KP, kd=rc.KD, initial=None,
                 ground=True, installation=False):
        b = newton.ModelBuilder()
        mount = (wp.transform(wp.vec3(*rm65.FIXED_MOUNT_XYZ),
                             wp.quat_from_axis_angle(wp.vec3(0,0,1), rm65.FIXED_MOUNT_RPY[2]))
                 if installation else wp.transform())
        b.add_urdf(str(rm65.URDF_PATH), floating=False, enable_self_collisions=True, xform=mount)
        rc.configure(b, kp, kd)
        if initial is not None:
            # One-time test initialization; never written during motion.
            for k,j in enumerate(rc.resolve_names(b.joint_label)):
                b.joint_q[b.joint_q_start[j]] = float(initial[k])
        b.gravity = gravity
        if ground:
            b.add_ground_plane()
        self.m = b.finalize(device='cuda')
        self.solver = newton.solvers.SolverMuJoCo(self.m, use_mujoco_contacts=False, nconmax=2048, njmax=8192)
        self.state, self.next = self.m.state(), self.m.state()
        newton.eval_fk(self.m, self.m.joint_q, self.m.joint_qd, self.state)
        self.control = self.m.control()
        self.contacts = self.m.contacts()
        self.frame = 0
        self.physics_steps = 0
        self.trace_substeps = False
        self.body_ids = {name: rm65._unique_named_index(self.m.body_label, name) for name in rm65.REQUIRED_LINK_NAMES}
        self.base_initial = self.state.body_q.numpy()[self.body_ids['base_link']].copy()
        self.audit_contacts(initial=True)
        # Reject installation geometry before any controller target write.
        self.controller = rc.JointController(self.m, self.control)
        self.controller.check_state(self.state)
        emit(event='candidate', kp=kp, kd=kd, running_effort=rc.EFFORT, gravity=gravity,
             control_hz=60, physics_substeps=3, physics_dt=1/180,
             effort_measurement='configured limits and backend clamp only; no measured effort curve')

    def audit_contacts(self, initial=False):
        m, s, c = self.m, self.state, self.contacts
        m.collide(s, c)
        count = int(c.rigid_contact_count.numpy()[0])
        a, z = c.rigid_contact_shape0.numpy()[:count], c.rigid_contact_shape1.numpy()[:count]
        p0, p1 = c.rigid_contact_point0.numpy()[:count], c.rigid_contact_point1.numpy()[:count]
        normals = c.rigid_contact_normal.numpy()[:count]
        sb, poses = m.shape_body.numpy(), s.body_q.numpy()
        pairs = {}
        audit_bodies = getattr(self, 'audit_body_ids', None)
        for i, (x, y) in enumerate(zip(a, z)):
            bx, by = int(sb[x]), int(sb[y])
            if audit_bodies is not None and bx not in audit_bodies and by not in audit_bodies:
                continue
            def world(body, p):
                return np.asarray(wp.transform_point(wp.transform(*poses[body]), wp.vec3(*p))) if body >= 0 else p
            distance = float(np.dot(world(by, p1[i])-world(bx, p0[i]), normals[i]))
            names = tuple(m.body_label[v].rsplit('/', 1)[-1] if v >= 0 else 'ground' for v in (bx, by))
            pair = '|'.join(names)
            pairs[pair] = min(pairs.get(pair, 1e9), distance)
        self.last_contact_distances = pairs
        penetrations = {pair: distance for pair, distance in pairs.items() if distance < -.0002}
        if penetrations:
            pair = min(penetrations, key=penetrations.get)
            raise UnsafePose(pair, penetrations[pair], self.physics_steps, penetrations)
        # Explicit finger/palm geometry even though direct parent pairs are filtered.
        palm = poses[self.body_ids['gripper_palm']]
        centers = []
        for name in ('gripper_left_finger', 'gripper_right_finger'):
            fp = poses[self.body_ids[name]]
            local = np.asarray(wp.transform_point(wp.transform_inverse(wp.transform(*palm)), wp.vec3(*fp[:3])))
            # Finger box lower z is its joint origin; palm upper surface z=.03.
            if local[2] < .03-.0002:
                raise RuntimeError(f'STOP: finger/palm penetration {name} {local}')
            centers.append(local)
        gap = centers[0][1]-centers[1][1]-.012
        if gap < -.0002:
            raise RuntimeError(f'STOP: finger/finger penetration {gap}')
        if initial:
            # Exclude visual-only shapes from this report: visual/collision
            # duplicate pairs must not be mistaken for disabled body pairs.
            flags = m.shape_flags.numpy()
            filtered = sorted(set(tuple(m.body_label[int(sb[i])].rsplit('/',1)[-1] if sb[i]>=0 else 'ground' for i in pair)
                                  for pair in m.shape_collision_filter_pairs
                                  if all(int(flags[i]) & int(newton.ShapeFlags.COLLIDE_SHAPES) for i in pair)))
            emit(event='initial_collision_audit', enabled_self_collisions=True, filtered_body_pairs=filtered,
                 candidate_pair_min_geometry_distance_m=pairs, finger_gap_m=gap, physics_steps=0)
        return gap

    def step_frame(self):
        self.controller.update(self.frame)
        samples = []
        target = self.control.joint_target_q.numpy().copy()
        for sub in range(3):
            self.state.clear_forces()
            self.m.collide(self.state, self.contacts)
            self.solver.step(self.state, self.next, self.control, self.contacts, 1/180)
            self.physics_steps += 1
            self.state, self.next = self.next, self.state
            if self.trace_substeps:
                c = self.controller
                emit(event='physics_sample', frame=self.frame, substep=sub,
                     physics_steps=self.physics_steps, time=self.physics_steps/180,
                     q=self.state.joint_q.numpy()[c.qs], qd=self.state.joint_qd.numpy()[c.ds],
                     raw_request=c.raw_arm_request, effective_target=c.effective_arm_target,
                     soft_clamped=c.soft_clamped, sent=c.sent,
                     control_target=c.control.joint_target_q.numpy()[c.ts])
            q, v = self.controller.check_state(self.state)
            self.audit_contacts()
            base = self.state.body_q.numpy()[self.body_ids['base_link']]
            if np.max(np.abs(base-self.base_initial)) > 1e-5:
                raise RuntimeError('STOP: fixed base moved')
            np.testing.assert_array_equal(self.control.joint_target_q.numpy(), target)
            samples.append((q.copy(), v.copy()))
        self.frame += 1
        return samples

    def segment(self, target, label, seconds=3.):
        c = self.controller
        start = self.state.joint_q.numpy()[c.qs].copy()
        clamped = c.set_arm_targets(target)
        samples = []
        consecutive, arrival = 0, None
        for _ in range(round(seconds*60)):
            for q, v in self.step_frame():
                samples.append((q,v))
                good = np.all(np.abs(q[:6]-c.goal[:6]) <= .001) and np.all(np.abs(v[:6]) <= .005)
                consecutive = consecutive+1 if good else 0
                if consecutive >= 45 and arrival is None:
                    arrival = len(samples)/180
        q = np.asarray([a for a,b in samples]); v = np.asarray([b for a,b in samples])
        delta = c.goal-start
        overshoot = np.maximum(0, (q-c.goal)*np.sign(delta)).max(axis=0)
        error = np.abs(q[-45:]-c.goal).max(axis=0)
        maximum_speed = np.abs(v).max(axis=0)
        emit(event='segment', label=label, target=c.goal, actual=q[-1], steady_error=error,
             overshoot=overshoot, max_speed=maximum_speed, arrival=arrival, clamped=clamped,
             raw_request=c.raw_arm_request, effective_target=c.effective_arm_target, soft_clamped=c.soft_clamped)
        if np.any(error[:6]>.001) or np.any(overshoot[:6]>.0025) or np.any(maximum_speed[:6]>.15) or arrival is None:
            raise ValueError('SAFE_PERFORMANCE_FAILURE: inspect recorded response before gain revision')
        return q, v


class UnsafePose(RuntimeError):
    def __init__(self, pair, distance, physics_steps, penetrations=None):
        self.pair, self.distance, self.physics_steps = pair, distance, physics_steps
        self.penetrations = penetrations or {pair: distance}
        super().__init__(f'STOP: penetration {pair} {distance:.9f} m; physics_steps={physics_steps}')


def mechanical_fixture(initial=None):
    """Mechanical-range-only fixture; no assertion of orchard installation safety."""
    f = Fixture(initial=initial, ground=False)
    if np.any(f.m.shape_body.numpy() < 0):
        raise AssertionError('mechanical fixture contains an environment shape')
    emit(event='mechanical_fixture', ground=False, trees=False, gravity=0,
         fixed_base=True, self_collisions=True, installation_safety_claim=False)
    return f


def run_c1():
    limits = rc.description()
    for axis, name in enumerate(rc.NAMES[:6]):
        for key, sign in (('lower', -1), ('upper', 1)):
            f = Fixture()
            c = f.controller
            requested = np.zeros(6)
            requested[axis] = limits[key][axis]+sign*.01
            assert c.set_arm_targets(requested)
            expected = limits[key][axis] - sign * rc.ARM_COMMAND_MARGIN
            assert c.goal[axis] == expected
            np.testing.assert_array_equal(c.raw_arm_request, requested)
            np.testing.assert_array_equal(c.effective_arm_target, c.goal[:6])
            assert c.soft_clamped[axis]
            before = c.control.joint_target_q.numpy().copy()
            # Every invalid request must preserve goal, sent target and buffer.
            goal, sent = c.goal.copy(), c.sent.copy()
            for invalid in ([0]*5, [float('nan')]*6, [float('inf')]*6):
                try:
                    c.set_arm_targets(invalid)
                except ValueError:
                    pass
                else:
                    raise AssertionError('invalid request accepted')
                np.testing.assert_array_equal(c.goal, goal)
                np.testing.assert_array_equal(c.sent, sent)
                np.testing.assert_array_equal(c.control.joint_target_q.numpy(), before)
            # Only one frame, three real substeps with identical target.
            # step_frame asserts buffer identity after each solver step.
            f.step_frame()
            expected = np.zeros(8); expected[axis] = sign*.1/60
            np.testing.assert_allclose(c.control.joint_target_q.numpy()[c.ts], expected, atol=1e-8)
            emit(stage='C1', joint=name, side=key, request=requested[axis],
                 clamped_goal=c.goal[axis], first_frame_target=c.sent[axis],
                 raw_request=c.raw_arm_request, effective_target=c.effective_arm_target, soft_clamped=c.soft_clamped,
                 shared_substeps=3, invalid_input_preserves_target=True, result='PASS')


def run_c2():
    initial = np.zeros(8); initial[1] = -2.258
    try:
        Fixture(initial=initial, installation=True, ground=True)
    except UnsafePose as exc:
        mount = [p for p in exc.penetrations if set(p.split('|')) == {'ground', 'gripper_mount'}]
        if len(mount) != 1 or exc.physics_steps != 0:
            raise
        emit(stage='C2', result='PASS', meaning='installation safety gate rejected unsafe pose',
             initial=initial, mount_xyz=rm65.FIXED_MOUNT_XYZ, mount_rpy=rm65.FIXED_MOUNT_RPY,
             collision_pair=mount[0], distance_m=exc.penetrations[mount[0]],
             all_penetrations_m=exc.penetrations, physics_steps=exc.physics_steps)
        return
    raise AssertionError('C2 dangerous installation pose was not rejected')


def run_c3():
    """Revised C3: joint2 soft-lower only; historical hard-limit runs retained."""
    limits = rc.description()
    initial = np.zeros(8)
    # Approach from .01 rad inside the new command boundary, not at rest on it.
    initial[1] = limits['lower'][1] + rc.ARM_COMMAND_MARGIN + .01
    f = mechanical_fixture(initial=initial)
    c = f.controller
    f.trace_substeps = True
    requested = np.zeros(6)
    requested[1] = limits['lower'][1] + rc.ARM_COMMAND_MARGIN
    assert abs(requested[1] - (-2.258)) < 1e-12
    assert not c.set_arm_targets(requested)
    minimum = np.full(8, np.inf)
    maximum = np.full(8, -np.inf)
    peak_speed = np.zeros(8)
    for frame in range(180):
        samples = f.step_frame()
        for sub, (q, v) in enumerate(samples):
            minimum = np.minimum(minimum, q)
            maximum = np.maximum(maximum, q)
            peak_speed = np.maximum(peak_speed, np.abs(v))
            # This reproduction additionally requires zero official arm crossing.
            # The controller's general hard-limit tolerance remains 0.0001 rad.
            if np.any(q[:6] < limits['lower'][:6]) or np.any(q[:6] > limits['upper'][:6]):
                raise RuntimeError('STOP: soft-lower revalidation crossed official hard limit')
    final, _ = c.read_arm_state(f.state)
    if abs(final[1] - requested[1]) > .001 or np.any(peak_speed[:6] > .15):
        raise ValueError('SAFE_PERFORMANCE_FAILURE: joint2 soft-lower')
    np.testing.assert_allclose(c.sent[:6], requested, atol=1e-12)
    emit(stage='C3', case='joint2-soft-lower', result='PASS',
         target=requested[1], initial=initial, minimum_q=minimum, maximum_q=maximum,
         max_speed=peak_speed, final_q=final, physics_steps=f.physics_steps,
         joint2_hard_lower_clearance=minimum[1]-limits['lower'][1],
         joint2_soft_overshoot=max(0., requested[1]-minimum[1]),
         meaning='soft command boundary validated; not a hard-boundary reach test')


def run_d():
    """Five seconds at the original zero home under normal gravity."""
    f = Fixture(gravity=-9.81)
    f.trace_substeps = True
    f.controller.set_arm_targets(np.zeros(6))
    records = []
    for frame in range(300):
        for sub, (q, v) in enumerate(f.step_frame()):
            records.append((q, v))
            emit(event='gravity_hold_sample', time=(frame*3+sub+1)/180, q=q, qd=v)
    q = np.asarray([x for x, _ in records]); v = np.asarray([x for _, x in records])
    error = np.abs(q[:, :6]).max(axis=0)
    late_speed = np.abs(v[-90:, :6]).max(axis=0)
    drift = np.ptp(q[-90:, :6], axis=0)
    emit(stage='D', event='metrics', duration_s=5, gravity=-9.81, home=np.zeros(6),
         max_error=error, max_speed=np.abs(v[:, :6]).max(axis=0),
         final_q=q[-1], late_speed=late_speed, late_peak_to_peak=drift,
         per_second_max_error=[np.abs(x[:, :6]).max(axis=0) for x in np.split(q, 5)],
         per_second_max_speed=[np.abs(x[:, :6]).max(axis=0) for x in np.split(v, 5)],
         position_tolerance=.001, settled_speed_tolerance=.005,
         tolerance_basis='retain B arm position/speed arrival tolerances; last 0.5 s = 90 physics samples')
    if np.any(error > .001) or np.any(late_speed > .005):
        raise ValueError('SAFE_PERFORMANCE_FAILURE: gravity hold; no automatic gain change')
    emit(stage='D', result='PASS', physics_steps=f.physics_steps)


def run_e():
    """Three small mixed-direction six-axis cycles under normal gravity."""
    f = Fixture(gravity=-9.81)
    f.trace_substeps = True
    target = np.array([.03, -.03, .02, -.02, .03, -.03])
    endings = []
    for repeat in range(3):
        for goal, tag in ((target, 'out'), (np.zeros(6), 'home')):
            start, _ = f.controller.read_arm_state(f.state)
            q, v = f.segment(goal, f'E:{repeat+1}:{tag}')
            late_speed = np.abs(v[-90:, :6]).max(axis=0)
            emit(stage='E', event='settled_speed', repeat=repeat+1, leg=tag, max_speed=late_speed)
            if np.any(late_speed > .005):
                raise ValueError('SAFE_PERFORMANCE_FAILURE: combined final-window velocity')
            moved = np.abs(goal-start) > .001
            if np.any((q[-1, :6]-start)[moved] * (goal-start)[moved] <= 0):
                raise RuntimeError('STOP: combined joint direction mismatch')
            emit(stage='E', event='direction', repeat=repeat+1, leg=tag,
                 desired_delta=goal-start, actual_delta=q[-1, :6]-start)
            endings.append(q[-1, :6])
    repeats = np.asarray(endings).reshape(3, 2, 6)
    spread = np.ptp(repeats, axis=0)
    if np.any(spread > .001):
        raise ValueError('SAFE_PERFORMANCE_FAILURE: combined repeatability')
    emit(stage='E', result='PASS', cycles=3, target=target, endpoint_spread=spread,
         gravity=-9.81, physics_steps=f.physics_steps)


def run_f():
    """Three unloaded open/half/closed/open cycles; no arm margin on fingers."""
    f = Fixture(gravity=-9.81)
    f.trace_substeps = True
    c = f.controller
    for repeat in range(3):
        for fraction in (1., .5, 0., 1.):
            c.set_gripper_open_fraction(fraction)
            requested = .06*(1-fraction)
            start, _ = c.read_gripper_state(f.state)
            records = []; gaps = []; symmetry = []; arrival = None; consecutive = 0
            # Full .06 m travel at .02 m/s needs 3 s; allow 1 s settling.
            for frame in range(240):
                for sub, (q, v) in enumerate(f.step_frame()):
                    records.append((q, v))
                    good = np.all(np.abs(q[6:]-requested) <= .0005) and np.all(np.abs(v[6:]) <= .001)
                    consecutive = consecutive+1 if good else 0
                    if consecutive >= 45 and arrival is None:
                        arrival = len(records)/180
                    emit(event='gripper_sample', repeat=repeat+1, fraction=fraction,
                         segment_time=(frame*3+sub+1)/180, q=q[6:], qd=v[6:], target=requested)
                poses = f.state.body_q.numpy()
                palm = wp.transform(*poses[f.body_ids['gripper_palm']])
                local = [np.asarray(wp.transform_point(wp.transform_inverse(palm), wp.vec3(*poses[f.body_ids[name]][:3])))
                         for name in ('gripper_left_finger', 'gripper_right_finger')]
                gap = local[0][1]-local[1][1]-.012
                sym = max(abs(local[0][0]-local[1][0]), abs(local[0][1]+local[1][1]), abs(local[0][2]-local[1][2]))
                gaps.append(gap); symmetry.append(sym)
                emit(event='gripper_geometry', repeat=repeat+1, fraction=fraction,
                     segment_time=(frame+1)/60, left_local=local[0], right_local=local[1], gap=gap, symmetry=sym)
            q = np.asarray([x for x, _ in records]); v = np.asarray([x for _, x in records])
            sync = np.max(np.abs(q[:, 6]-q[:, 7]))
            error = np.abs(q[-90:, 6:]-requested).max(axis=0)
            late_speed = np.abs(v[-90:, 6:]).max(axis=0)
            expected_gap = .12*fraction
            emit(stage='F', event='segment', repeat=repeat+1, fraction=fraction, target=requested,
                 actual=q[-1, 6:], sync_error=sync, steady_error=error,
                 max_speed=np.abs(v[:, 6:]).max(axis=0), late_speed=late_speed,
                 arrival=arrival, min_gap=min(gaps), final_gap=gaps[-1], expected_gap=expected_gap,
                 max_geometry_asymmetry=max(symmetry), position_tolerance=.0005, speed_tolerance=.001)
            moving = np.abs(requested-start) > .0005
            if np.any((q[-1, 6:]-start)[moving] * (requested-start)[moving] <= 0):
                raise RuntimeError('STOP: gripper joint direction mismatch')
            if abs(gaps[-1]-expected_gap) > .001 or max(symmetry) > .0005:
                raise RuntimeError('STOP: gripper geometry/semantics mismatch')
            if sync > .0005 or np.any(error > .0005) or np.any(late_speed > .001) or arrival is None:
                raise ValueError('SAFE_PERFORMANCE_FAILURE: gripper symmetry/settling')
    emit(stage='F', result='PASS', cycles=3, physics_steps=f.physics_steps)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--stage', choices=['B', 'C1', 'C2', 'C3', 'D', 'E', 'F'], required=True)
    args=p.parse_args()
    if args.stage == 'B':
        f=Fixture()
        for axis,name in enumerate(rc.NAMES[:6]):
            for value in (.05,0.,-.05,0.):
                target=np.zeros(6); target[axis]=value
                f.segment(target, f'{name}:{value:+.2f}')
    else:
        {'C1': run_c1, 'C2': run_c2, 'C3': run_c3, 'D': run_d, 'E': run_e, 'F': run_f}[args.stage]()
    emit(stage=args.stage, result='PASS')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        emit(result='STOP', error=str(exc))
        raise
