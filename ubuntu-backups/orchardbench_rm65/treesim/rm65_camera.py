"""RM fixed-mount synthetic depth observation, without motion or picking.

Uses the original Newton sensor and FruitPerception. The bracket is a virtual
Stage 3.4 design, not a calibrated D435. Camera axes: +X right, +Y up, -Z forward.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp
from newton.sensors import SensorTiledCamera
from .perception import FruitPerception
from .rm65_kinematics import pose_from_xyz_quat

TRANSLATION = (0.11, 0.0, 0.03)
LOOK_AT = (0.0, 0.0, 0.38)
INSTALLATION_SOURCE = 'stage34_virtual_equivalent'
# Calculate once: camera +Y is exactly parent +Y; camera -Z aims at LOOK_AT.
_z = np.asarray(TRANSLATION) - np.asarray(LOOK_AT)
_z /= np.linalg.norm(_z)
QUATERNION_XYZW = tuple(Rotation.from_matrix(
    np.column_stack((np.cross([0., 1., 0.], _z), [0., 1., 0.], _z))).as_quat())


class CameraBinding:
    def __init__(self, robot_data, env=0, parent_link='gripper_mount',
                 translation=TRANSLATION, quaternion_xyzw=QUATERNION_XYZW):
        if (robot_data.get('model'), robot_data.get('mount')) != ('rm65', 'fixed'):
            raise ValueError('camera binding requires rm65 + fixed')
        if parent_link != 'gripper_mount' or not isinstance(env, int) or env < 0:
            raise ValueError('invalid RM camera parent/environment')
        maps = robot_data.get('body_indices', {})
        if parent_link not in maps or any(env >= len(v) for v in maps.values()):
            raise ValueError('missing camera parent/environment mapping')
        ids = [v[env] for v in maps.values()]
        if any(not isinstance(i, (int, np.integer)) or i < 0 for i in ids) or len(set(ids)) != len(ids):
            raise ValueError('duplicate or invalid RM body mapping')
        self.parent_body = int(maps[parent_link][env])
        self.self_bodies = frozenset(int(i) for i in ids)
        self.T_parent_camera = pose_from_xyz_quat(translation, quaternion_xyzw)
        self.T_parent_camera.setflags(write=False)
        self.source = INSTALLATION_SOURCE

    def world_pose(self, body_poses):
        values = np.asarray(body_poses)
        if values.ndim != 2 or values.shape[1] != 7 or max(self.self_bodies) >= len(values):
            raise ValueError('body pose mapping out of range')
        p = values[self.parent_body].astype(float)
        # Newton float32 FK can accumulate tiny quaternion normalization error.
        if not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1.) > 1e-5:
            raise ValueError('invalid parent pose')
        return pose_from_xyz_quat(p[:3], p[3:]/np.linalg.norm(p[3:])) @ self.T_parent_camera

    def filter_depth(self, depth, shape_index, shape_body):
        depth, hits, sb = np.asarray(depth), np.asarray(shape_index), np.asarray(shape_body)
        if depth.shape != hits.shape or not np.issubdtype(hits.dtype, np.integer):
            raise ValueError('depth and hit IDs must have identical sample shape')
        valid = hits != 0xffffffff
        if np.any(hits[valid] < 0) or np.any(hits[valid] >= len(sb)):
            raise ValueError('unresolved ray-hit shape ID')
        own = np.zeros(hits.shape, dtype=bool)
        own[valid] = np.isin(sb[hits[valid].astype(int)], list(self.self_bodies))
        masked = depth.astype(float, copy=True)
        masked[own] = np.nan
        return masked, own


@dataclass(frozen=True)
class CameraSample:
    source: str
    sample_id: int
    simulation_time: float
    T_world_camera: np.ndarray
    depth: np.ndarray
    shape_index: np.ndarray
    self_mask: np.ndarray
    detection_depth: np.ndarray
    detections_world: tuple


def create_rm_camera(model, viewer, tree, params):
    """Disabled mode must not construct or initialize any rendering resources."""
    if not getattr(params, 'rm_camera', False):
        return None
    if (params.model, params.mount, params.rm_control) != ('rm65', 'fixed', 'joint'):
        raise ValueError('RM camera requires fixed joint observation')
    if not params.camera or not tree.config.fruit.enabled or tree.num_envs != 1:
        raise ValueError('RM camera requires explicit camera, apples and one environment')
    return RMSyntheticCamera(model, viewer, tree, params)


class RMSyntheticCamera:
    def __init__(self, model, viewer, tree, params, *, binding=None):
        if tree.num_envs != 1:
            raise ValueError('RM observation supports one environment')
        self.model, self.viewer = model, viewer
        self.binding = binding or CameraBinding(tree.robot_data, env=0)
        self.every = int(params.camera_every)
        if self.every < 1:
            raise ValueError('camera cadence must be positive')
        self.range = float(params.camera_range)
        self.sensor = SensorTiledCamera(model)
        W, H = int(params.camera_width), int(params.camera_height)
        self.rays = self.sensor.utils.compute_pinhole_camera_rays(W, H, math.radians(params.camera_fov))
        self.depth = self.sensor.utils.create_depth_image_output(W, H, 1)
        self.shape_index = self.sensor.utils.create_shape_index_image_output(W, H, 1)
        self.percept = FruitPerception(W, H, params.camera_fov)
        self.shape_body = model.shape_body.numpy().copy()
        self.last_sample = None
        self.T_world_camera = None
        self._frame = -1
        self._time = -1.
        self._sample_frame = None

    def update(self, state, simulation_time, frame):
        """Return a new complete sample or None, never a cached 'new detection'.

        At 60 Hz/every=12 this is 5 Hz. Pausing or a repeated call at the same
        physics frame never increments sample_id. Sample arrays own their data.
        """
        t = float(simulation_time)
        if not np.isfinite(t) or t < 0 or not isinstance(frame, int) or frame < 0:
            raise ValueError('invalid simulation clock')
        if frame < self._frame or t < self._time or (frame == self._frame and t != self._time):
            raise ValueError('non-monotonic simulation clock')
        self.T_world_camera = self.binding.world_pose(state.body_q.numpy())
        if frame == self._frame:
            return None
        self._frame, self._time = frame, t
        if self._sample_frame is not None and frame-self._sample_frame < self.every:
            return None
        pose = self.T_world_camera.copy()
        transform = wp.transform(wp.vec3(*pose[:3, 3]), wp.quat(*Rotation.from_matrix(pose[:3,:3]).as_quat()))
        cameras = wp.array([[transform]], dtype=wp.transform, device=self.model.device)
        self.model.bvh_refit_shapes(state)
        self.sensor.update(state, cameras, self.rays, depth_image=self.depth,
                           shape_index_image=self.shape_index,
                           clear_data=SensorTiledCamera.ClearData(clear_depth=self.range))
        depth = self.depth.numpy()[0, 0].copy()
        hits = self.shape_index.numpy()[0, 0].copy()
        masked, own = self.binding.filter_depth(depth, hits, self.shape_body)
        detections = tuple(self.percept.detect(masked, pose[:3,3], pose[:3,:3]))
        for array in (pose, depth, hits, masked, own):
            array.setflags(write=False)
        sample = CameraSample('orchardbench_synthetic',
                              0 if self.last_sample is None else self.last_sample.sample_id+1,
                              t, pose, depth, hits, own, masked, detections)
        self.last_sample = sample
        self._sample_frame = frame
        if self.viewer is not None:
            gray = (255 * (1-np.clip(depth/self.range, 0, 1))).astype(np.uint8)
            rgba = np.dstack((gray, gray, gray, np.full_like(gray, 255)))
            rgba = self.percept.draw_overlay(rgba, detections, masked)
            self.viewer.log_image('RM synthetic wrist depth', rgba[None])
        return sample
