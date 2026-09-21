#!/usr/bin/env python3
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class Grabber(Node):
    def __init__(self, out_dir: Path):
        super().__init__('grab_branch_node')
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.img = None
        self.depth = None
        self.cinfo = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.on_color, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.on_depth, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.on_info, 10)

    def on_color(self, msg: Image):
        if self.img is None:
            self.img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    def on_depth(self, msg: Image):
        if self.depth is None:
            self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))

    def on_info(self, msg: CameraInfo):
        if self.cinfo is None:
            self.cinfo = {
                'width': msg.width,
                'height': msg.height,
                'k': list(msg.k)
            }

def main():
    rclpy.init()
    out = Path('/home/li/vision_experiments/2026-09-09_branch_occlusion/data')
    node = Grabber(out)
    t0 = time.time()
    while (node.img is None or node.depth is None or node.cinfo is None) and (time.time() - t0 < 4.0):
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.img is not None and node.depth is not None and node.cinfo is not None:
        cv2.imwrite(str(out / 'branch_color.jpg'), cv2.cvtColor(node.img, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / 'branch_depth.png'), node.depth)
        with open(out / 'camera_info.json', 'w') as f:
            json.dump(node.cinfo, f, indent=2)
        print("GRAB_SUCCESS")
    else:
        print(f"GRAB_FAILED: img={node.img is not None}, depth={node.depth is not None}, cinfo={node.cinfo is not None}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
