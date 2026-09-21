"""Task-scale acceptance policy for RM65 fixed-base empty-hand validation.

Only the exact base collision shape / world ground plane installation pair is
exempt from 5 mm clearance. It still obeys the original 0.2 mm penetration gate.
Distances use an independent MuJoCo model/data with Newton FK body transforms;
no live q, qd, body poses, targets, collision filters or dynamics are written.
"""
import copy
import numpy as np
import mujoco
import newton
from scipy.spatial.transform import Rotation
from .rm65_kinematics import pose_error
from .rm65_planning import PlanningFailure

class EnvironmentClearance:

    def __init__(self, r):
        cfg = r.tree.config.robot
        if (cfg.model, cfg.mount) != ('rm65', 'fixed'):
            raise ValueError('SCOPE')
        s = r.sim.solver
        self.m = copy.copy(s.mj_model)
        self.d = mujoco.MjData(self.m)
        self.n = r.sim.model
        self.sb = self.n.shape_body.numpy()
        self.gmap = s.mjc_geom_to_newton_shape.numpy()[0].astype(int)
        self.bmap = s.mjc_body_to_newton.numpy()[0].astype(int)
        self.bids = self.bmap[self.m.geom_bodyid]
        self.localR = Rotation.from_quat(self.m.geom_quat[:, [1, 2, 3, 0]]).as_matrix()
        self.robot = set(r.audit.body_ids.values())
        self.base = r.audit.body_ids['base_link']
        flags = self.n.shape_flags.numpy()
        collision = set(np.flatnonzero(flags & int(newton.ShapeFlags.COLLIDE_SHAPES)))
        if collision != set(self.gmap):
            raise RuntimeError('DISTANCE_INTERFACE: incomplete geom mapping')
        self.rg = np.array([i for i, b in enumerate(self.bids) if b in self.robot])
        self.eg = np.array([i for i, b in enumerate(self.bids) if b not in self.robot])
        grounds = [i for i in self.eg if self.sb[self.gmap[i]] == -1 and self.n.shape_label[self.gmap[i]] == 'ground_plane' and (self.m.geom_type[i] == int(mujoco.mjtGeom.mjGEOM_PLANE))]
        if len(grounds) != 1:
            raise RuntimeError('DISTANCE_INTERFACE: exact ground plane unresolved')
        self.ground = grounds[0]
        base_geoms = [i for i in self.rg if self.sb[self.gmap[i]] == self.base]
        if len(base_geoms) != 1:
            raise RuntimeError('DISTANCE_INTERFACE: base collision shape unresolved')
        self.install = (base_geoms[0], self.ground)
        self.pairs = [(int(a), int(b)) for a in self.rg for b in self.eg if (a, b) != self.install]
        self.a = np.array([p[0] for p in self.pairs])
        self.b = np.array([p[1] for p in self.pairs])
        self.plane = self.m.geom_type[self.b] == int(mujoco.mjtGeom.mjGEOM_PLANE)
        self.minimum = None
        self.install_min = float('inf')
        self.install_max = -float('inf')
        self.samples = 0
        self.d.qpos[:] = s.mjw_data.qpos.numpy()[0]
        mujoco.mj_kinematics(self.m, self.d)
        xp = self.d.geom_xpos.copy()
        xr = self.d.geom_xmat.copy()
        self.update(r.sim.state_0)
        np.testing.assert_allclose(self.d.geom_xpos, xp, atol=2e-06, rtol=0)
        np.testing.assert_allclose(self.d.geom_xmat, xr, atol=2e-06, rtol=0)

    def update(self, state):
        """Populate isolated geom poses; never write the supplied Newton state."""
        poses = state.body_q.numpy()
        R = Rotation.from_quat(poses[:, 3:]).as_matrix()
        ids = self.bids
        valid = ids >= 0
        self.d.geom_xpos[:] = self.m.geom_pos
        self.d.geom_xmat[:] = self.localR.reshape(-1, 9)
        self.d.geom_xpos[valid] = poses[ids[valid], :3] + np.einsum('nij,nj->ni', R[ids[valid]], self.m.geom_pos[valid])
        self.d.geom_xmat[valid] = (R[ids[valid]] @ self.localR[valid]).reshape(-1, 9)

    def distance(self, a, b):
        endpoints = np.zeros(6)
        value = float(mujoco.mj_geomDistance(self.m, self.d, int(a), int(b), 100.0, endpoints))
        if not np.isfinite(value) or value >= 100:
            raise RuntimeError('DISTANCE_INTERFACE: unsupported/capped pair')
        return value

    def row(self, a, b, d, phase):
        nb = self.sb[self.gmap[b]]
        return dict(distance_m=d, robot_link=self.n.body_label[self.sb[self.gmap[a]]], environment_object=self.n.body_label[nb] if nb >= 0 else self.n.shape_label[self.gmap[b]], robot_shape=int(self.gmap[a]), environment_shape=int(self.gmap[b]), phase=phase)

    def check(self, state, phase):
        """Check every forbidden pair, pruning only by conservative sphere bounds.

  Planes are never sphere-pruned. A pair is skipped only when its lower bound
  exceeds an exact distance already measured in this sample. Distances refer
  to MuJoCo collision geometry (including its convex mesh representation), not
  contact counts. Discrete samples do not prove continuous path clearance.
  """
        self.update(state)
        self.samples += 1
        install = self.distance(*self.install)
        self.install_min = min(self.install_min, install)
        self.install_max = max(self.install_max, install)
        if install < -0.0002:
            raise RuntimeError('INSTALLATION_PENETRATION ' + str(self.row(*self.install, install, phase)))
        lower = np.linalg.norm(self.d.geom_xpos[self.a] - self.d.geom_xpos[self.b], axis=1) - self.m.geom_rbound[self.a] - self.m.geom_rbound[self.b]
        lower[self.plane] = -np.inf
        best = float('inf')
        row = None
        for k in np.argsort(lower):
            if lower[k] > best:
                break
            a, b = self.pairs[k]
            v = self.distance(a, b)
            if v < best:
                best = v
                row = self.row(a, b, v, phase)
            if v < 0.005:
                raise RuntimeError('ENVIRONMENT_CLEARANCE ' + str(row))
        if self.minimum is None or best < self.minimum['distance_m']:
            self.minimum = row
        return (row, install)

def arrived(ep, er, v):
    return bool(np.asarray(v).shape == (6,) and np.isfinite(ep) and np.isfinite(er) and np.all(np.isfinite(v)) and (ep <= 0.005) and (er <= np.deg2rad(1)) and (np.max(np.abs(v)) <= 0.005))

def repeatability(endpoints):
    result = {}
    for label in ('pre_grasp', 'grasp', 'retract', 'home'):
        rows = [r for r in endpoints if r['stage'] == label]
        if len(rows) != 3:
            raise PlanningFailure('INCOMPLETE', label)
        ps = []
        rs = []
        for i in range(3):
            for j in range(i):
                p, r = pose_error(np.array(rows[i]['actual_pose_world']), np.array(rows[j]['actual_pose_world']))
                ps.append(p)
                rs.append(r)
        pe = [r['position_error'] for r in rows]
        re = [r['rotation_error'] for r in rows]
        increasing = all((pe[i + 1] > pe[i] for i in range(2))) or all((re[i + 1] > re[i] for i in range(2)))
        result[label] = dict(position_spread_m=max(ps), rotation_spread_rad=max(rs), position_errors=pe, rotation_errors=re, monotonic_increase=increasing)
        if max(ps) > 0.002 or max(rs) > np.deg2rad(0.25) or increasing:
            raise PlanningFailure('REPEATABILITY', result)
    return result

class ArrivalWindow:
    """45 consecutive physics samples at the required 180 Hz; failures reset."""

    def __init__(self):
        self.count = 0

    def reset(self):
        self.count = 0

    def update(self, position_error, rotation_error, velocity):
        self.count = self.count + 1 if arrived(position_error, rotation_error, velocity) else 0
        return self.count >= 45
