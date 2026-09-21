"""Conservative observed-surface geometry. No ROS, hardware or execution imports.

Free means observed free along a depth ray, never certified robot clearance.
All distances are metres; depth values use the explicitly recorded scale.
"""
import hashlib
import json
import itertools

import numpy as np

PREFIX = 'auto_obstacle_'


def _positive(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f'invalid {name}')
    return value


def rigid_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-5)):
        raise ValueError('invalid rigid camera_to_base')
    return t


def _intrinsics(intr):
    fx, fy = _positive(intr['fx'], 'fx'), _positive(intr['fy'], 'fy')
    cx, cy = float(intr['ppx']), float(intr['ppy'])
    if not np.isfinite([cx, cy]).all():
        raise ValueError('invalid principal point')
    # Pinhole projection is valid only for rectified / zero-distortion data.
    if np.any(np.asarray(intr.get('coeffs', [0.]), float) != 0):
        raise ValueError('nonzero distortion requires explicit rectification')
    return fx, fy, cx, cy


def validate_observation(record, now_s, max_age_s):
    stamp = float(record['capture_stamp_s'])
    if not np.isfinite([stamp, now_s, max_age_s]).all() or max_age_s < 0:
        raise ValueError('invalid capture clock')
    if stamp > now_s + .05 or now_s - stamp > max_age_s:
        raise ValueError('stale_or_future_capture')
    scale = _positive(record['depth_scale_m'], 'depth_scale_m')
    if scale > 1:
        raise ValueError('implausible depth unit scale')
    _intrinsics(record['intrinsics'])
    ds = float(record['depth_stamp_s'])
    if not np.isfinite(ds) or abs(ds - stamp) > .035:
        raise ValueError('unsynchronized_rgbd')
    if record.get('color_frame') != record.get('depth_frame') or not record.get('color_frame'):
        raise ValueError('depth_not_aligned_to_color')
    if record.get('frame_id') == 'base_link':
        rigid_transform(record.get('camera_to_base'))
        span, bracket = float(record.get('joint_span_deg', np.nan)), float(record.get('bracket_s', np.nan))
        if (record.get('stationary') is not True or not np.isfinite([span, bracket]).all()
                or not 0 <= span <= .02 or not 0 < bracket <= .6):
            raise ValueError('robot_not_stationary_or_unbracketed')
        prov = record.get('transform_provenance', {})
        if prov.get('synchronized') is not True or not prov.get('source'):
            raise ValueError('missing_transform_provenance')
        if len(record.get('robot_model_sha256', '')) != 64:
            raise ValueError('missing_robot_model_hash')
        joints = np.asarray(record.get('joint_positions_rad', []), float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError('invalid_recorded_joints')
    elif record.get('frame_id') != record.get('color_frame'):
        raise ValueError('unsupported_frame')


def depth_points(depth, intrinsics, scale_m, transform=None):
    depth = np.asarray(depth)
    if depth.ndim != 2:
        raise ValueError('depth must be H x W')
    scale = _positive(scale_m, 'depth_scale_m')
    fx, fy, cx, cy = _intrinsics(intrinsics)
    v, u = np.nonzero(np.isfinite(depth) & (depth > 0))
    z = depth[v, u].astype(float) * scale
    p = np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z))
    if transform is not None:
        t = rigid_transform(transform)
        p = p @ t[:3, :3].T + t[:3, 3]
    return p


def _limits(points, config):
    keep = np.ones(len(points), bool)
    if config.get('bounds_m') is not None:
        bounds = np.asarray(config['bounds_m'], float)
        if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]):
            raise ValueError('invalid bounds_m')
        keep &= np.all((points >= bounds[0]) & (points <= bounds[1]), axis=1)
    return keep


def build_snapshot(depth, record, config, target_mask=None):
    # Replay validates internal consistency at observation time. Online age is
    # checked again at the ROS boundary, against the actual wall clock.
    validate_observation(record, record['capture_stamp_s'], 0.)
    depth = np.asarray(depth)
    intr = record['intrinsics']
    if depth.shape != (intr['height'], intr['width']):
        raise ValueError('depth_intrinsics_size_mismatch')
    t = record.get('camera_to_base') if record['frame_id'] == 'base_link' else None
    points = depth_points(depth, intr, record['depth_scale_m'], t)
    valid = np.isfinite(depth) & (depth > 0)
    z = depth[valid].astype(float)*record['depth_scale_m']
    near = _positive(config.get('min_depth_m', .1), 'min_depth_m')
    far = _positive(config.get('max_depth_m', 2.), 'max_depth_m')
    if near >= far:
        raise ValueError('invalid depth range')
    keep = (z >= near) & (z <= far) & _limits(points, config)
    points = points[keep]
    resolution = _positive(config.get('voxel_size_m', .012), 'voxel_size_m')
    padding = float(config.get('padding_m', .004))
    if not np.isfinite(padding) or padding < 0:
        raise ValueError('invalid padding_m')
    indices = np.floor(points / resolution).astype(np.int64)
    cells = np.unique(indices, axis=0)
    if len(cells) > int(config.get('max_cells', 30000)):
        raise ValueError('too_many_cells_no_truncation_permitted')
    def cell_id(idx):
        return PREFIX + '_'.join(str(int(v)) for v in idx)
    occupied = {cell_id(i): ((i+.5)*resolution).tolist() for i in cells}
    target_ids = []
    if target_mask is not None:
        mask = np.asarray(target_mask, bool)
        if mask.shape != depth.shape:
            raise ValueError('target_mask_size_mismatch')
        target_ids = sorted({cell_id(i) for i in indices[mask[valid][keep]]})
    limitations = ['single_view_unknown_space', 'calibration_error_unmeasured',
                   'robot_envelope_unverified', 'surface_model_no_semantic_contact_permission']
    if t is None:
        limitations.append('missing_camera_to_base')
    if not occupied:
        limitations.append('no_valid_observed_geometry')
    return dict(schema_version=1, frame_id=record['frame_id'], units='m',
                capture_stamp_s=record['capture_stamp_s'], observation=record,
                transform_provenance=record.get('transform_provenance', {}),
                robot_model_sha256=record.get('robot_model_sha256'),
                occupied_cells=occupied, voxel_size_m=resolution, box_size_m=resolution+2*padding,
                unknown_policy='zero_depth_outside_frustum_behind_surface_and_outside_roi_are_unknown',
                target_geometry={'observed_cell_ids': target_ids, 'retained_as_collision': True,
                                 'mask_is_semantic_only': True},
                quality={'valid_depth_fraction': float(valid.mean()), 'point_count': len(points),
                         'cell_count': len(cells), 'limitations': limitations},
                config=config, executable=False)


def _camera_points(points, record):
    p = np.asarray(points, float).reshape(-1, 3)
    if record['frame_id'] == 'base_link':
        t = rigid_transform(record['camera_to_base'])
        p = (p-t[:3, 3]) @ t[:3, :3]
    return p


def classify_points(points, depth, record, config):
    """Conservative nearest-pixel ray test; invalid neighbors stay unknown."""
    original = np.asarray(points, float).reshape(-1, 3)
    p = _camera_points(original, record)
    fx, fy, cx, cy = _intrinsics(record['intrinsics'])
    out = np.full(len(p), 'unknown', dtype='<U8')
    tol = _positive(config.get('surface_tolerance_m', .012), 'surface_tolerance_m')
    margin = int(config.get('pixel_guard', 1))
    if margin < 0:
        raise ValueError('negative pixel_guard')
    allowed = _limits(original, config)
    h, w = depth.shape
    for i, (x, y, z) in enumerate(p):
        if (not allowed[i] or not np.isfinite([x, y, z]).all()
                or z < config.get('min_depth_m', .1) or z > config.get('max_depth_m', 2.)):
            continue
        u, v = int(np.rint(fx*x/z+cx)), int(np.rint(fy*y/z+cy))
        if u-margin < 0 or v-margin < 0 or u+margin >= w or v+margin >= h:
            continue
        patch = depth[v-margin:v+margin+1, u-margin:u+margin+1]*record['depth_scale_m']
        if (not np.isfinite(patch).all() or np.any(patch < config.get('min_depth_m', .1))
                or np.any(patch > config.get('max_depth_m', 2.))):
            continue
        closest = float(patch.min())
        if abs(z-closest) <= tol:
            out[i] = 'occupied'
        elif z < closest-tol:
            out[i] = 'free'
    return out


def cleared_cells(previous, depth, record, config):
    """An entire old AABB must project into valid rays strictly in front of depth.

    Checking just its center or corners could erase a thin branch inside it.
    The full projected pixel rectangle deliberately over-approximates the box.
    """
    if previous.get('frame_id') != record['frame_id']:
        return set()
    half = _positive(previous['box_size_m'], 'box_size_m')/2
    offsets = np.array(list(itertools.product([-half, half], repeat=3)))
    fx, fy, cx, cy = _intrinsics(record['intrinsics'])
    tol = _positive(config.get('surface_tolerance_m', .012), 'surface_tolerance_m')
    h, w = depth.shape; result = set()
    for ident, center in previous['occupied_cells'].items():
        if not ident.startswith(PREFIX):
            continue
        corners = np.asarray(center)+offsets
        if not _limits(corners, config).all():
            continue
        p = _camera_points(corners, record)
        if not np.isfinite(p).all() or p[:, 2].min() < config.get('min_depth_m', .1):
            continue
        uv = p[:, :2]/p[:, 2, None]*[fx, fy]+[cx, cy]
        lo = np.floor(uv.min(axis=0)).astype(int); hi = np.ceil(uv.max(axis=0)).astype(int)
        if lo[0] < 0 or lo[1] < 0 or hi[0] >= w or hi[1] >= h:
            continue
        patch = depth[lo[1]:hi[1]+1, lo[0]:hi[0]+1]*record['depth_scale_m']
        if (np.isfinite(patch).all() and np.all(patch > 0)
                and patch.max() <= config.get('max_depth_m', 2.)
                and patch.min() > p[:, 2].max()+tol):
            result.add(ident)
    return result


def scene_delta(previous, current, cleared_ids):
    for field in ('frame_id', 'voxel_size_m', 'box_size_m'):
        if field in previous and field in current and previous[field] != current[field]:
            raise ValueError(f'incompatible_scene_{field}')
    old, new = previous['occupied_cells'], current['occupied_cells']
    remove = sorted(i for i in old if i.startswith(PREFIX) and i not in new and i in cleared_ids)
    add = {i: c for i, c in new.items() if i.startswith(PREFIX) and (i not in old or old[i] != c)}
    return {'add': add, 'remove': remove}


def canonical_scene(objects):
    """Stable digest of complete caller-supplied object geometry, order independent."""
    encoded = sorted(json.dumps(obj, sort_keys=True, separators=(',', ':'), allow_nan=False) for obj in objects)
    return hashlib.sha256(('['+','.join(encoded)+']').encode()).hexdigest()
