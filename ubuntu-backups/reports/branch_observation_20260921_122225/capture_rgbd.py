"""Current RGB-D observation only: no robot or gripper interfaces."""
import json
import time
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
import pyrealsense2 as rs

started = datetime.now().astimezone()
out = Path('/home/li/桌面/9月机械臂实验（算法）/reports') / ('branch_observation_' + started.strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
records = []
active = False
error = None
try:
    profile = pipeline.start(config)
    active = True
    align = rs.align(rs.stream.color)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    deadline = time.monotonic() + 18
    count = 0
    while time.monotonic() < deadline and len(records) < 3:
        frames = pipeline.wait_for_frames(timeout_ms=3000)
        count += 1
        if count < 30 or count % 20:
            continue
        frames = align.process(frames)
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            continue
        index = len(records)
        color_path, depth_path = out / f'rgb_{index:02d}.jpg', out / f'depth_{index:02d}.npy'
        if not cv2.imwrite(str(color_path), np.asanyarray(color.get_data()), [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError('RGB save failed')
        depth_array = np.asanyarray(depth.get_data())
        np.save(depth_path, depth_array)
        intr = color.profile.as_video_stream_profile().intrinsics
        records.append({'received_at': datetime.now().astimezone().isoformat(), 'color_frame_number': color.get_frame_number(), 'color_timestamp_ms': color.get_timestamp(), 'depth_timestamp_ms': depth.get_timestamp(), 'timestamp_domain': str(color.get_frame_timestamp_domain()), 'depth_scale_m': scale, 'depth_valid_fraction': float(np.mean(depth_array > 0)), 'intrinsics': {'width': intr.width, 'height': intr.height, 'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy, 'distortion_model': str(intr.model), 'coeffs': list(intr.coeffs)}, 'rgb_path': str(color_path), 'depth_path': str(depth_path)})
    if len(records) != 3:
        raise RuntimeError('Insufficient fresh observations')
except Exception as exc:
    error = repr(exc)
    raise
finally:
    if active:
        pipeline.stop()
    metadata = {'started_at': started.isoformat(), 'ended_at': datetime.now().astimezone().isoformat(), 'source': 'Newly started local RealSense stream; no robot access', 'error': error, 'records': records}
    (out / 'capture.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(json.dumps({'output_directory': str(out), 'frames': len(records), 'error': error, 'records': records}, ensure_ascii=False))
