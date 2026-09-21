"""RM-only runtime safety and independent scene preflight; no physics replacement."""
import copy
import json
import numpy as np
import newton
import warp as wp
from . import rm65, rm65_control as rc


def emit(**data):
    print(json.dumps(data, default=lambda a: a.tolist() if isinstance(a, np.ndarray) else float(a)), flush=True)


class GeometryAudit:
    """Stage 3.2 signed-distance and finger geometry gates, scoped to RM bodies."""
    def __init__(self, model, state):
        self.m, self.state = model, state
        self.contacts = model.contacts()
        self.physics_steps = 0
        self.body_ids = {name: rm65._unique_named_index(model.body_label, name)
                         for name in rm65.REQUIRED_LINK_NAMES}
        self.audit_body_ids = set(self.body_ids.values())

    def check_no_contact(self):
        self.audit_contacts()
        touching = {p:d for p,d in self.last_contact_distances.items()
                    if d <= 0. and frozenset(p.split('|')) not in getattr(self, 'allowed_touch_pairs', set())}
        if touching:
            raise RuntimeError(f"COLLISION: {touching}")

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


class UnsafePose(RuntimeError):
    def __init__(self, pair, distance, physics_steps, penetrations=None):
        self.pair, self.distance, self.physics_steps = pair, distance, physics_steps
        self.penetrations = penetrations or {pair: distance}
        super().__init__(f'STOP: penetration {pair} {distance:.9f} m; physics_steps={physics_steps}')



class RMRuntime:
    """One target update per frame, safety after every real physics substep."""
    def __init__(self, tree, sim):
        if (tree.config.robot.model, tree.config.robot.mount, tree.config.robot.rm_control) != ('rm65','fixed','joint'):
            raise ValueError('RM runtime requires rm65+fixed+joint')
        self.picking = bool(getattr(tree.config.robot, 'rm_auto_baseline', False))
        self.close_only = None  # explicit program capability; default remains open-hand
        self.allowed_pick_index = -1
        self.observation = bool(getattr(tree.config.robot, 'rm_camera', False))
        if tree.num_envs != 1:
            raise ValueError('RM runtime prohibits multiple environments')
        if self.observation:
            if not tree.config.robot.camera or not tree.config.fruit.enabled or sim.breaker is not None:
                raise ValueError('RM observation requires camera/apples and prohibits branch breaking')
        elif tree.config.robot.camera or tree.config.fruit.enabled:
            raise ValueError('RM runtime prohibits cameras/apples without explicit observation')
        if sim.substeps != 3 or abs(sim.frame_dt-1/60)>1e-12:
            raise ValueError('RM runtime requires 60 Hz / 3 substeps')
        solver = sim.solver
        if int(solver.mj_model.opt.solver) != 1 or int(solver.mjw_model.opt.solver) != 1:
            raise ValueError('RM runtime requires CPU/GPU CG')
        self.tree, self.sim = tree, sim
        if not all(np.isfinite(a.numpy()).all() for a in
                   (sim.state_0.joint_q,sim.state_0.joint_qd,sim.state_0.body_q,sim.state_0.body_qd)):
            raise RuntimeError('SAFETY: nonfinite initial state')
        js = rc.resolve_names(sim.model.joint_label)
        initial_q = sim.state_0.joint_q.numpy()[sim.model.joint_q_start.numpy()[js]]
        if np.any(np.abs(initial_q[6:]) > .0005):
            raise RuntimeError('SAFETY: Stage 3.3 requires initially open fingers')
        self.audit = GeometryAudit(sim.model, sim.state_0)
        self.audit.audit_contacts(initial=True)
        self.audit.check_no_contact()
        if np.any(np.abs(initial_q[:6]) > 1e-7):
            raise RuntimeError("SAFETY: Stage 3.3 startup requires measured home")
        # Geometry rejection MUST precede JointController's first target write.
        self.controller = rc.JointController(sim.model, sim.control)
        self.controller.check_state(sim.state_0)
        self.base_initial = sim.state_0.body_q.numpy()[self.audit.body_ids['base_link']].copy()
        self.controller.set_arm_targets(np.zeros(6))
        self.controller.set_gripper_open_fraction(1.)
        self.frames = self.substeps = 0
        self.expected_target = None
        self.max_error = self.max_speed = self.max_base_error = 0.
        self.listeners = []
        self.trace = False
        self.stopped = False
        # Only this RM instance disables capture; the original Sim loop is retained.
        sim._capture = lambda: None
        sim._graph = None
        runtime = self
        class AuditedSolver:
            def __getattr__(self, name):
                return getattr(solver, name)
            def step(self, state_in, state_out, control, contacts, dt):
                if runtime.stopped:
                    raise RuntimeError("SAFETY: runtime already stopped")
                try:
                    solver.step(state_in, state_out, control, contacts, dt)
                    runtime.substeps += 1
                    if runtime.close_only is not None:
                        runtime.close_only.observe_contacts(state_out)
                    if runtime.trace:
                        emit(event='rm_substep', step=runtime.substeps,
                             q=state_out.joint_q.numpy()[runtime.controller.qs],
                             qd=state_out.joint_qd.numpy()[runtime.controller.ds],
                             target=control.joint_target_q.numpy()[runtime.controller.ts])
                    q,v = runtime.controller.check_state(state_out)
                    runtime.audit.state = state_out
                    runtime.audit.physics_steps = runtime.substeps
                    # This stage has no intended contact with self/environment.
                    runtime.audit.check_no_contact()
                    base = state_out.body_q.numpy()[runtime.audit.body_ids['base_link']]
                    base_error = float(np.max(np.abs(base-runtime.base_initial)))
                    if base_error > 1e-5:
                        raise RuntimeError('SAFETY: fixed base moved')
                    np.testing.assert_array_equal(control.joint_target_q.numpy(), runtime.expected_target)
                    runtime.max_error = max(runtime.max_error, float(np.max(np.abs(q[:6]-runtime.controller.goal[:6]))))
                    runtime.max_speed = max(runtime.max_speed, float(np.max(np.abs(v[:6]))))
                    runtime.max_base_error = max(runtime.max_base_error, base_error)
                    for listener in runtime.listeners:
                        listener(state_out)
                except Exception as exc:
                    runtime.stopped = True
                    if runtime.close_only is not None:
                        runtime.close_only.fail(exc)
                    emit(event='rm_stop', category='RUNTIME_SAFETY', step=runtime.substeps, error=str(exc))
                    raise
        sim.solver = AuditedSolver()
        emit(event='rm_runtime_ready', solver='MuJoCo+CG', hz=60, substeps=3,
             camera=self.observation, apples=self.observation, picker=False,
             initial_check_before_controller=True)

    def before_frame(self):
        if self.observation and not self.picking and self.sim.apples is not None:
            if np.any(self.sim.apples._held_host) or np.any(self.sim.apples.detached):
                self.stopped = True
                raise RuntimeError('SAFETY: observation forbids apple hold/detachment')
        if self.stopped:
            raise RuntimeError('SAFETY: runtime already stopped')
        if self.picking:
            a = self.sim.apples
            active = np.flatnonzero((a._held_host != 0) | np.asarray(a.detached, dtype=bool))
            if any(int(i) != self.allowed_pick_index for i in active):
                self.stopped = True
                raise RuntimeError('SAFETY: non-target apple hold/detachment')
        close_authorized = self.close_only is not None and self.close_only.closing_authorized()
        if not self.picking and not close_authorized and np.any(self.controller.goal[6:] != 0.):
            self.stopped = True
            raise RuntimeError('SAFETY: Stage 3.3 fingers must remain open')
        self.controller.update(self.frames)
        self.expected_target = self.sim.control.joint_target_q.numpy().copy()

    def after_frame(self):
        if self.observation and not self.picking and self.sim.apples is not None:
            if np.any(self.sim.apples._held_host) or np.any(self.sim.apples.detached):
                self.stopped = True
                raise RuntimeError('SAFETY: observation apple hold/detachment detected')
        self.frames += 1
        if self.substeps != self.frames*3:
            raise RuntimeError('SAFETY: physics substep count mismatch')

    def snapshot(self):
        return dict(frames=self.frames, substeps=self.substeps, max_error=self.max_error,
                    max_speed=self.max_speed, max_base_error=self.max_base_error)


class ScratchScene:
    """Separate model/state/contact pipeline for RM paths in a scene snapshot.

    No write access to the live model, state or controller. Discrete samples
    do not prove continuous collision freedom; execution retains substep gates.
    """
    def __init__(self, runtime):
        from . import builder
        cfg = copy.deepcopy(runtime.tree.config)
        tree = builder.build(cfg, copy.deepcopy(runtime.tree.skeleton),
                             max_bodies=len(runtime.tree.skeleton), num_envs=1)
        self.model = tree.model
        live = runtime.sim.model
        if list(self.model.body_label) != list(live.body_label) or list(self.model.joint_label) != list(live.joint_label):
            raise RuntimeError('PREFLIGHT: scratch scene topology mismatch')
        self.state = self.model.state()
        self.state.assign(runtime.sim.state_0)
        self.snapshot_q = runtime.sim.state_0.joint_q.numpy().copy()
        self.q = wp.array(self.snapshot_q, dtype=wp.float32, device=self.model.device)
        self.qd = wp.zeros(self.model.joint_dof_count, device=self.model.device)
        js = rc.resolve_names(self.model.joint_label)
        self.qs = self.model.joint_q_start.numpy()[js]
        starts, ends = self.model.articulation_start.numpy()[:-1], self.model.articulation_end.numpy()
        ids = np.flatnonzero((starts <= js[0]) & (ends > js[0]))
        if len(ids) != 1:
            raise RuntimeError('PREFLIGHT: ambiguous RM articulation')
        self.indices = wp.array(ids.astype(np.int32), dtype=wp.int32, device=self.model.device)
        self.audit = GeometryAudit(self.model, self.state)
        self.samples = 0

    def check(self, q_arm):
        q_arm = rc._vector(q_arm, 6)
        limits = rc.description()
        if np.any(q_arm < limits['lower'][:6]+rc.ARM_COMMAND_MARGIN) or np.any(q_arm > limits['upper'][:6]-rc.ARM_COMMAND_MARGIN):
            raise ValueError('LIMIT: scratch command outside soft bounds')
        q = self.snapshot_q.copy(); q[self.qs[:6]] = q_arm; q[self.qs[6:]] = 0.
        self.q.assign(q)
        newton.eval_fk(self.model, self.q, self.qd, self.state, indices=self.indices)
        self.audit.check_no_contact()
        self.samples += 1
        return True

    def check_slew(self, start, end):
        q, end = rc._vector(start,6).copy(), rc._vector(end,6)
        self.check(q)
        for _ in range(10000):
            if np.array_equal(q,end):
                return
            q += np.clip(end-q, -rc.RATE[:6]/60., rc.RATE[:6]/60.)
            self.check(q)
        raise RuntimeError('TIMEOUT: scratch slew budget')
