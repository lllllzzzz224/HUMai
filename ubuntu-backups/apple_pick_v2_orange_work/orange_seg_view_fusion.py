import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class OrangeSegView(Node):
    def __init__(self):
        super().__init__('orange_seg_multiscale')
        self.bridge = CvBridge()
        self.tracks = []
        self.max_missed = 3

        self.model = YOLO('/home/li/hand_eye_calibration/models/yolo11n-seg.pt')

        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.callback, 10
        )

        cv2.namedWindow('Orange Vision', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Orange Vision', 1100, 800)

    def detect(self, img, ox=0, oy=0):
        result = self.model.predict(
            img,
            classes=[49],
            conf=0.15,
            imgsz=960,
            verbose=False
        )[0]

        detections = []

        if result.boxes is None:
            return detections

        for i, box in enumerate(result.boxes):
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            conf = float(box.conf[0])

            mask_pts = None
            if result.masks is not None and i < len(result.masks.xy):
                pts = np.asarray(result.masks.xy[i], dtype=np.int32)
                if len(pts) >= 3:
                    pts[:, 0] += ox
                    pts[:, 1] += oy
                    mask_pts = pts

            detections.append([
                x1 + ox, y1 + oy,
                x2 + ox, y2 + oy,
                conf, mask_pts
            ])

        return detections


    def temporal_stabilize(self, detections):
        # detections: [x1, y1, x2, y2, conf, pts]
        matched_tracks = set()
        output = []

        for det in detections:
            x1, y1, x2, y2, conf, pts = det

            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            size = max(x2 - x1, y2 - y1)

            best = -1
            best_dist = 1e9

            for i, tr in enumerate(self.tracks):
                if i in matched_tracks:
                    continue

                tx1, ty1, tx2, ty2, _, _ = tr["det"]
                tcx = (tx1 + tx2) * 0.5
                tcy = (ty1 + ty2) * 0.5

                dist = ((cx-tcx)**2 + (cy-tcy)**2)**0.5
                tsize = max(tx2-tx1, ty2-ty1)

                # 保守的位置匹配，避免把旁边另一颗橘子接过来
                limit = 0.30 * max(size, tsize)

                if dist < limit and dist < best_dist:
                    best = i
                    best_dist = dist

            if best >= 0:
                old = self.tracks[best]["det"]
                matched_tracks.add(best)

                # 轻度位置平滑；新检测占75%
                a = 0.75
                sx1 = int(a*x1 + (1-a)*old[0])
                sy1 = int(a*y1 + (1-a)*old[1])
                sx2 = int(a*x2 + (1-a)*old[2])
                sy2 = int(a*y2 + (1-a)*old[3])

                stable = [sx1, sy1, sx2, sy2, conf, pts]
                output.append(stable)
            else:
                output.append(det)

        new_tracks = []

        # 当前真实检测进入缓存
        for det in output:
            new_tracks.append({"det": det, "missed": 0})

        # 短暂漏检最多保留3帧
        for i, tr in enumerate(self.tracks):
            if i in matched_tracks:
                continue

            missed = tr["missed"] + 1

            if missed <= self.max_missed:
                output.append(tr["det"])
                new_tracks.append({
                    "det": tr["det"],
                    "missed": missed
                })

        self.tracks = new_tracks
        return output

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        detections = self.detect(frame)

        # 3x3 overlapping local crops for heavy leaf occlusion
        cw = int(w * 0.55)
        ch = int(h * 0.55)

        xs = [0, int((w-cw)/2), w-cw]
        ys = [0, int((h-ch)/2), h-ch]

        for y1 in ys:
            for x1 in xs:
                x2 = x1 + cw
                y2 = y1 + ch
                crop = frame[y1:y2, x1:x2]
                detections.extend(self.detect(crop, x1, y1))

        detections = self.temporal_stabilize(detections)

        if not detections:
            cv2.imshow('Orange Vision', frame)
            cv2.waitKey(1)
            return

        boxes = []
        scores = []

        for d in detections:
            x1, y1, x2, y2 = d[:4]
            boxes.append([
                int(x1), int(y1),
                int(x2-x1), int(y2-y1)
            ])
            scores.append(float(d[4]))

        # Candidate Fusion：合并同一颗橘子的重复检测
        def box_iou(a, b):
            ax, ay, aw, ah = a
            bx, by, bw, bh = b
            ax2, ay2 = ax + aw, ay + ah
            bx2, by2 = bx + bw, by + bh

            ix1, iy1 = max(ax, bx), max(ay, by)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            iw = max(0, ix2 - ix1)
            ih = max(0, iy2 - iy1)

            inter = iw * ih
            union = aw * ah + bw * bh - inter
            return inter / union if union > 0 else 0.0

        order = sorted(
            range(len(boxes)),
            key=lambda i: scores[i],
            reverse=True
        )

        keep = []
        suppressed = set()

        for i in order:
            if i in suppressed:
                continue

            keep.append(i)

            x, y, w0, h0 = boxes[i]
            cx = x + w0 / 2.0
            cy = y + h0 / 2.0

            for j in order:
                if j == i or j in suppressed:
                    continue

                x2, y2, w2, h2 = boxes[j]
                cx2 = x2 + w2 / 2.0
                cy2 = y2 + h2 / 2.0

                dist = ((cx-cx2)**2 + (cy-cy2)**2)**0.5
                scale = min(max(w0, h0), max(w2, h2))
                iou = box_iou(boxes[i], boxes[j])

                # 小框包含率：解决同一颗橘子出现“完整框 + 半个小框”
                ax, ay, aw, ah = boxes[i]
                bx, by, bw, bh = boxes[j]

                ix1 = max(ax, bx)
                iy1 = max(ay, by)
                ix2 = min(ax + aw, bx + bw)
                iy2 = min(ay + ah, by + bh)

                inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                smaller = min(aw * ah, bw * bh)
                contain = inter / smaller if smaller > 0 else 0.0

                if iou > 0.28 or contain > 0.65 or dist < 0.30 * scale:
                    suppressed.add(j)

        keep = np.asarray(keep, dtype=np.int32)

        display = frame.copy()

        if len(keep) > 0:
            for idx in np.array(keep).reshape(-1):
                x1, y1, x2, y2, conf, pts = detections[int(idx)]

                x1, y1 = int(x1), int(y1)
                x2, y2 = int(x2), int(y2)

                if pts is not None:
                    overlay = display.copy()
                    cv2.fillPoly(overlay, [pts], (0,255,255))
                    display = cv2.addWeighted(
                        overlay, 0.25,
                        display, 0.75, 0
                    )
                    cv2.polylines(
                        display, [pts], True,
                        (0,255,0), 2
                    )

                cv2.rectangle(
                    display,
                    (x1,y1),(x2,y2),
                    (255,0,255),2
                )

                cv2.putText(
                    display,
                    f'orange {conf:.2f}',
                    (x1,max(25,y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255,0,255),2
                )

        cv2.imshow('Orange Vision', display)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = OrangeSegView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
