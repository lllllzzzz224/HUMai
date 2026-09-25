#!/usr/bin/env python3
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class SyncCollector(Node):
    def __init__(self, output_dir: Path, target_count: int = 20):
        super().__init__('sync_collector')
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_count = target_count
        self.saved_count = 0
        self.camera_info = None
        self.latest_color = None
        self.latest_depth = None
        self.records = []

        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.on_color, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.on_depth, 10)
        self.create_timer(0.05, self.check_and_save)

    def on_info(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = {
                'width': msg.width,
                'height': msg.height,
                'k': list(msg.k),
                'd': list(msg.d),
                'distortion_model': msg.distortion_model,
                'frame_id': msg.header.frame_id
            }
            with open(self.output_dir / 'camera_info.json', 'w') as f:
                json.dump(self.camera_info, f, indent=2)
            self.get_logger().info(f'CameraInfo saved: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}')

    def on_color(self, msg: Image):
        self.latest_color = msg

    def on_depth(self, msg: Image):
        self.latest_depth = msg

    def check_and_save(self):
        if self.saved_count >= self.target_count:
            return
        if self.camera_info is None or self.latest_color is None or self.latest_depth is None:
            return

        c_stamp = self.latest_color.header.stamp.sec * 1e9 + self.latest_color.header.stamp.nanosec
        d_stamp = self.latest_depth.header.stamp.sec * 1e9 + self.latest_depth.header.stamp.nanosec
        diff_ms = abs(c_stamp - d_stamp) / 1e6
        if diff_ms > 35.0:  # must be within one frame interval (~33ms)
            return

        color_data = np.frombuffer(self.latest_color.data, dtype=np.uint8).reshape((self.latest_color.height, self.latest_color.width, 3))
        depth_data = np.frombuffer(self.latest_depth.data, dtype=np.uint16).reshape((self.latest_depth.height, self.latest_depth.width))

        idx = self.saved_count
        c_path = self.output_dir / f'frame_{idx:03d}_color.jpg'
        d_path = self.output_dir / f'frame_{idx:03d}_depth.png'

        cv2.imwrite(str(c_path), cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(d_path), depth_data)

        self.records.append({
            'index': idx,
            'color_file': c_path.name,
            'depth_file': d_path.name,
            'sync_error_ms': diff_ms,
            'color_stamp_sec': self.latest_color.header.stamp.sec,
            'color_stamp_nanosec': self.latest_color.header.stamp.nanosec,
            'depth_stamp_sec': self.latest_depth.header.stamp.sec,
            'depth_stamp_nanosec': self.latest_depth.header.stamp.nanosec,
        })
        self.saved_count += 1
        self.get_logger().info(f'Saved frame {idx + 1}/{self.target_count} (sync err: {diff_ms:.2f} ms)')

        # clear to wait for next frame
        self.latest_color = None
        self.latest_depth = None

        if self.saved_count >= self.target_count:
            with open(self.output_dir / 'metadata.json', 'w') as f:
                json.dump(self.records, f, indent=2)
            self.get_logger().info('Data collection complete!')

def main():
    rclpy.init()
    out = Path('/home/li/vision_experiments/2026-09-08_occlusion_test/data')
    node = SyncCollector(out, target_count=20)
    start_time = time.time()
    while rclpy.ok() and node.saved_count < node.target_count:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > 10.0:
            node.get_logger().warn('Timeout waiting for frames')
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
