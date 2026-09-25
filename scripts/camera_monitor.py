#!/usr/bin/env python3
"""
RM65 实时视觉监控大屏 - 机械臂与果实识别实况画面 (GUI & Web 流)
------------------------------------------------------------
支持:
  1. Ubuntu 物理显示屏独立窗口显示 (X11 / Wayland)
  2. Web 浏览器实时流 (http://10.77.0.2:5000)
"""

import argparse
import glob
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# 全局最新帧缓存与锁
latest_jpeg = None
jpeg_lock = threading.Lock()


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # 禁用标准输出访问日志

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            html = """<!DOCTYPE html>
<html>
<head>
    <title>RM65 机械臂采摘实时视觉监控</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { margin: 0; background: #121212; color: #fff; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 100vh; }
        .container { text-align: center; max-width: 960px; width: 100%; padding: 10px; box-sizing: border-box; }
        h1 { margin: 10px 0; font-size: 20px; color: #4ade80; }
        .stream-box { border-radius: 8px; overflow: hidden; box-shadow: 0 4px 20px rgba(0,0,0,0.6); display: inline-block; background: #000; }
        img { width: 100%; max-width: 800px; height: auto; display: block; }
        .footer { margin-top: 10px; font-size: 13px; color: #888; }
    </style>
</head>
<body>
    <div class="container">
        <h1>RM65 实时视觉监控 (RealSense D435 + YOLOv11)</h1>
        <div class="stream-box">
            <img src="/stream.mjpg" alt="Live Stream" />
        </div>
        <div class="footer">实时监控流 | 包含 YOLO 识别框、3D 果心度量与拟合误差</div>
    </div>
</body>
</html>"""
            self.wfile.write(html.encode("utf-8"))
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with jpeg_lock:
                        frame = latest_jpeg
                    if frame is not None:
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.04)  # ~25 FPS
            except Exception:
                pass


class CameraMonitorNode(Node):
    def __init__(self):
        super().__init__("camera_monitor_node")
        self.latest_annotated = None
        self.latest_raw = None
        self.target_pos = None
        self.diagnostics = {}
        self.fps_counter = 0
        self.fps = 0.0
        self.last_fps_time = time.time()

        # 订阅话题
        self.create_subscription(Image, "/apple_pick_v2/annotated_image", self.on_annotated, 2)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_raw, 2)
        self.create_subscription(PoseStamped, "/apple_pick_v2/apple_center", self.on_center, 10)
        self.create_subscription(String, "/apple_pick_v2/apple_diagnostics", self.on_diag, 10)

    def on_annotated(self, msg: Image):
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self.latest_annotated = data
        self._update_fps()

    def on_raw(self, msg: Image):
        if self.latest_annotated is None:
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.latest_raw = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            self._update_fps()

    def on_center(self, msg: PoseStamped):
        p = msg.pose.position
        self.target_pos = (p.x, p.y, p.z)

    def on_diag(self, msg: String):
        try:
            self.diagnostics = json.loads(msg.data)
        except Exception:
            pass

    def _update_fps(self):
        self.fps_counter += 1
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.fps_counter / (now - self.last_fps_time)
            self.fps_counter = 0
            self.last_fps_time = now

    def render_display_frame(self) -> np.ndarray:
        frame = self.latest_annotated
        if frame is None:
            frame = self.latest_raw
        if frame is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for Camera / Vision Nodes...", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            return blank

        disp = frame.copy()
        h, w = disp.shape[:2]

        # 1. 顶部状态条 (半透明蒙版)
        overlay = disp.copy()
        cv2.rectangle(overlay, (0, 0), (w, 36), (20, 20, 20), -1)
        # 底部状态条
        cv2.rectangle(overlay, (0, h - 32), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, disp, 0.25, 0, disp)

        # 顶部文字
        cv2.putText(disp, "RM65 VISION MONITOR", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (74, 222, 128), 2, cv2.LINE_AA)
        status_txt = f"FPS: {self.fps:.1f} | ROS Domain: 42"
        cv2.putText(disp, status_txt, (w - 210, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

        # 底部文字：果心与拟合参数
        if self.target_pos is not None:
            px, py, pz = self.target_pos
            r_mm = self.diagnostics.get("radius_m", 0.0) * 1000.0
            conf = self.diagnostics.get("confidence", 0.0)
            bot_txt = f"Center: [{px:.3f}, {py:.3f}, {pz:.3f}]m | R={r_mm:.1f}mm | Conf={conf:.2f}"
            cv2.putText(disp, bot_txt, (12, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(disp, "Searching target fruit...", (12, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        return disp


def main():
    parser = argparse.ArgumentParser(description="RM65 实时相机监控大屏")
    parser.add_argument("--gui", action="store_true", default=True, help="在物理屏幕打开独立窗口")
    parser.add_argument("--no-gui", dest="gui", action="store_false", help="不打开物理桌面窗口")
    parser.add_argument("--web", action="store_true", default=True, help="开启 Web 实时流 (默认开启)")
    parser.add_argument("--port", type=int, default=5000, help="Web 端口 (默认 5000)")
    args = parser.parse_args()

    # 配置物理显示环境变量
    if args.gui:
        if "DISPLAY" not in os.environ:
            os.environ["DISPLAY"] = ":0"
        if "WAYLAND_DISPLAY" not in os.environ:
            os.environ["WAYLAND_DISPLAY"] = "wayland-0"
        if "XDG_RUNTIME_DIR" not in os.environ:
            os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"
        auths = glob.glob("/run/user/1000/.mutter-Xwaylandauth.*")
        if auths and "XAUTHORITY" not in os.environ:
            os.environ["XAUTHORITY"] = auths[0]

    rclpy.init()
    node = CameraMonitorNode()

    # 启动 ROS 2 订阅后台线程
    def ros_spin():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # 启动 Web 服务器后台线程
    if args.web:
        try:
            httpd = ThreadedHTTPServer(("0.0.0.0", args.port), MJPEGStreamHandler)
            web_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
            web_thread.start()
            print(f"\n[INFO] Web 实时监控已启动: http://10.77.0.2:{args.port}")
        except Exception as e:
            print(f"[WARN] Web 服务器启动失败: {e}")

    # 物理 GUI 窗口管理
    window_created = False
    if args.gui:
        try:
            cv2.namedWindow("RM65 RealSense Monitor", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("RM65 RealSense Monitor", 960, 720)
            window_created = True
            print("[INFO] 物理桌面窗口已创建，快捷键: 'q' 或 'ESC' 退出, 'f' 全屏")
        except Exception as e:
            print(f"[WARN] 无法打开物理 GUI 窗口: {e}")
            window_created = False

    is_fullscreen = False
    global latest_jpeg

    try:
        while True:
            disp_frame = node.render_display_frame()

            # 编码 JPEG 供 Web 流使用
            ret, jpeg = cv2.imencode(".jpg", disp_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret:
                with jpeg_lock:
                    latest_jpeg = jpeg.tobytes()

            # 更新物理桌面窗口
            if window_created:
                cv2.imshow("RM65 RealSense Monitor", disp_frame)
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord('q'), ord('Q')):
                    break
                elif key in (ord('f'), ord('F')):
                    is_fullscreen = not is_fullscreen
                    if is_fullscreen:
                        cv2.setWindowProperty("RM65 RealSense Monitor", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    else:
                        cv2.setWindowProperty("RM65 RealSense Monitor", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            else:
                time.sleep(0.04)

    except KeyboardInterrupt:
        pass
    finally:
        if window_created:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("\n[INFO] 监控已退出。")


if __name__ == "__main__":
    main()
