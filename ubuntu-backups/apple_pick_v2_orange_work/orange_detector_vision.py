import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

from ultralytics import YOLO


class OrangeDetector(Node):

    def __init__(self):

        super().__init__('orange_detector')

        self.bridge = CvBridge()

        self.model = YOLO('yolo11n.pt')

        self.depth_image = None


        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )


        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )


        self.get_logger().info(
            'Orange Detector Started!'
        )


    def depth_callback(self, msg):

        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )



    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )


        results = self.model.predict(
            source=frame,
            classes=[49],
            conf=0.2,
            imgsz=640,
            verbose=False
        )


        result = results[0]


        if result.boxes is not None:

            for box in result.boxes:


                x1,y1,x2,y2 = map(
                    int,
                    box.xyxy[0]
                )


                conf = float(
                    box.conf[0]
                )


                # 橘子中心点

                u = int((x1+x2)/2)
                v = int((y1+y2)/2)


                Z = 0
                X = 0
                Y = 0


                if self.depth_image is not None:


                    h,w = self.depth_image.shape


                    if (
                        0 <= u < w and
                        0 <= v < h
                    ):


                        Z = int(
                            self.depth_image[v,u]
                        )


                        if Z > 0:

                            # D435 RGB内参

                            fx = 615.0
                            fy = 615.0

                            cx = 320.0
                            cy = 240.0


                            X = int(
                                (u-cx)*Z/fx
                            )


                            Y = int(
                                (v-cy)*Z/fy
                            )



                text = (
                    f"orange {conf:.2f}\n"
                    f"orange {conf:.2f}"
                )


                cv2.rectangle(
                    frame,
                    (x1,y1),
                    (x2,y2),
                    (255,0,255),
                    2
                )


                cv2.putText(
                    frame,
                    text,
                    (x1,y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255,0,255),
                    2
                )



        cv2.imshow(
            "YOLO Orange Detection",
            frame
        )

        cv2.waitKey(1)



def main():

    rclpy.init()


    node = OrangeDetector()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()
