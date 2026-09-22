#!/usr/bin/env python3
"""Offline RGB-D replay and surface export. Never opens hardware or ROS."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import yaml

from obstacle_geometry import build_snapshot, depth_points, rigid_transform, _limits


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalize_record(capture, index, depth_shape):
    raw = capture['records'][index]
    if 'intrinsics' in raw:
        intr = dict(raw['intrinsics'])
    else:
        k = raw['K']
        intr = dict(fx=k[0], fy=k[4], ppx=k[2], ppy=k[5], width=depth_shape[1],
                    height=depth_shape[0], coeffs=raw.get('D', []))
    color_stamp = raw.get('rgb_stamp', raw.get('color_timestamp_ms', 0)/1000.)
    depth_stamp = raw.get('depth_stamp', raw.get('depth_timestamp_ms', 0)/1000.)
    if not color_stamp or not depth_stamp:
        raise ValueError('capture timestamp missing')
    scale = raw.get('depth_scale_m')
    if scale is None:
        if raw.get('depth_encoding') != '16UC1':
            raise ValueError('depth units missing')
        scale = .001  # realsense2_camera aligned 16UC1 publishes millimetres
    color_frame = raw.get('color_frame', 'camera_color_optical_frame')
    out = dict(capture_stamp_s=color_stamp, depth_stamp_s=depth_stamp, depth_scale_m=scale,
               intrinsics=intr, frame_id=color_frame, color_frame=color_frame,
               depth_frame=raw.get('depth_frame', color_frame),
               depth_scale_source='SDK metadata' if 'depth_scale_m' in raw else 'realsense2_camera 16UC1 mm contract',
               source_record_index=index)
    if raw.get('camera_to_base') is not None:
        pre, post = raw['pre_arm'], raw['post_arm']
        qa = np.asarray(pre['data']['arm_state']['joint'], float)
        qb = np.asarray(post['data']['arm_state']['joint'], float)
        if qa.shape != (6,) or qb.shape != (6,) or not np.isfinite([qa, qb]).all():
            raise ValueError('invalid controller joints')
        for sample in (pre, post):
            if sample['data'].get('state') != 'current_arm_state' or sample['data']['arm_state'].get('err') != [0]:
                raise ValueError('controller read error')
        times = [pre['start_wall'], pre['end_wall'], min(color_stamp, depth_stamp),
                 max(color_stamp, depth_stamp), post['start_wall'], post['end_wall']]
        if not np.isfinite(times).all() or any(a > b for a, b in zip(times, times[1:])):
            raise ValueError('capture not bracketed by controller reads')
        if not capture.get('hand_eye_source_sha256') or not capture.get('urdf_sha256'):
            raise ValueError('missing_transform_source_hashes')
        t = rigid_transform(raw['camera_to_base'])
        composed = (rigid_transform(raw['base_to_Link6']) @ rigid_transform(capture['hand_eye'])
                    @ rigid_transform(raw['camera_link_to_optical']))
        if not np.allclose(t, composed, atol=1e-7):
            raise ValueError('transform_chain_mismatch')
        span = float(np.abs(qa-qb).max()/1000)
        out.update(frame_id='base_link', camera_to_base=t.tolist(), stationary=span <= .02,
                   joint_span_deg=span, bracket_s=times[-1]-times[0],
                   joint_positions_rad=np.deg2rad((qa+qb)/2000).tolist(),
                   robot_model_sha256=capture['urdf_sha256'],
                   transform_provenance=dict(synchronized=True,
                       source='controller bracket + URDF FK + recorded handeye + camera optical TF',
                       hand_eye_source_sha256=capture['hand_eye_source_sha256'],
                       pre_read=pre, post_read=post,
                       base_to_Link6=raw['base_to_Link6'], hand_eye=capture['hand_eye'],
                       camera_link_to_optical=raw['camera_link_to_optical']))
    return out


def write_ply(path, points):
    with Path(path).open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\n'
                 f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode())
        f.write(np.asarray(points, dtype='<f4').tobytes())


def run_preview(input_path, output_path, config):
    start = time.monotonic()
    source, output = Path(input_path).resolve(), Path(output_path).resolve()
    if output == source.parent or output in source.parents:
        raise ValueError('output must be separate from source evidence')
    if output.exists() and any(output.iterdir()):
        raise ValueError('refusing to overwrite preview evidence')
    capture = json.loads(source.read_text())
    if capture.get('error') or not capture.get('records'):
        raise ValueError('failed_or_empty_capture')
    index = int(config.get('record_index', len(capture['records'])-1))
    if not 0 <= index < len(capture['records']):
        raise ValueError('record_index out of bounds')
    raw = capture['records'][index]
    depth_path = Path(raw.get('depth_path', f'depth_{index:02d}.npy'))
    if not depth_path.is_absolute():
        depth_path = source.parent/depth_path
    depth = np.load(depth_path, allow_pickle=False)
    record = normalize_record(capture, index, depth.shape)
    record.update(capture_file=str(source), capture_sha256=sha256(source),
                  depth_file=str(depth_path), depth_sha256=sha256(depth_path))
    effective = dict(config)
    if record['frame_id'] != 'base_link':
        effective['bounds_m'] = effective.pop('camera_bounds_m', None)
    mask = np.load(config['target_mask'], allow_pickle=False) if config.get('target_mask') else None
    snapshot = build_snapshot(depth, record, effective, mask)
    points = depth_points(depth, record['intrinsics'], record['depth_scale_m'], record.get('camera_to_base'))
    z = depth[np.isfinite(depth) & (depth > 0)]*record['depth_scale_m']
    keep = (z >= effective.get('min_depth_m', .1)) & (z <= effective.get('max_depth_m', 2.)) & _limits(points, effective)
    snapshot['quality'].update(processing_s=time.monotonic()-start,
                               source_capture_errors=capture.get('errors', []),
                               output_kind='recorded_surface_preview',
                               source_rgb=str(source.parent/f'rgb_{index:02d}.jpg'))
    snapshot['scene_version'] = hashlib.sha256(json.dumps(snapshot, sort_keys=True, allow_nan=False).encode()).hexdigest()
    output.mkdir(parents=True, exist_ok=False)
    write_ply(output/'observation.ply', points[keep])
    np.savez_compressed(output/'observation.npz', xyz_m=points[keep])
    (output/'scene.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2, allow_nan=False))
    (output/'quality.json').write_text(json.dumps(snapshot['quality'] | {'executable': False}, ensure_ascii=False, indent=2))
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_suffix('.yaml'))
    parser.add_argument('--target-mask', type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.target_mask:
        config['target_mask'] = str(args.target_mask.resolve())
    s = run_preview(args.capture, args.output, config)
    print(json.dumps({'frame_id': s['frame_id'], 'cells': len(s['occupied_cells']),
                      'output': str(args.output), 'executable': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
