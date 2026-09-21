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

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        detections = self.detect(frame)

        # 4个重叠局部区域
        overlap = 0.15
        cw = int(w * 0.62)
        ch = int(h * 0.62)

        regions = [
            (0, 0, cw, ch),
            (w-cw, 0, w, ch),
            (0, h-ch, cw, h),
            (w-cw, h-ch, w, h)
        ]

        for x1, y1, x2, y2 in regions:
            crop = frame[y1:y2, x1:x2]
            detections.extend(self.detect(crop, x1, y1))

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

        # 合并全图和局部检测产生的重复橘子
        keep = cv2.dnn.NMSBoxes(
            boxes, scores,
            score_threshold=0.15,
            nms_threshold=0.45
        )

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
