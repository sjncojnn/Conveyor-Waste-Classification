#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class WasteDetectorClassifier(Node):
    def __init__(self):
        super().__init__('waste_detector_classifier')
        self.bridge = CvBridge()

        # locate your models
        share = get_package_share_directory('yolobot_recognition')
        det_weights = os.path.join(share, 'scripts', 'Detect_YOLOv8.pt')
        cls_weights = os.path.join(share, 'scripts', 'Classi_YOLOv8(1).pt')

        # 1) detection model
        self.det_model = YOLO(det_weights)
        # 2) classification model
        self.cls_model = YOLO(cls_weights)

        # 3) map class names → pusher indices
        self.class_to_pusher = {
            'Plastic':            1,
            'Organic_Food_Waste': 2,
            'Paper_Cardboard':    3,
            'Metal_Other':        4,
            # 'Glass': # no pusher
        }

        # 4) subscribe camera; publish pusher index (delayed)
        self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10
        )
        self.pusher_pub = self.create_publisher(UInt8, '/pusher_index', 10)

        # ------------------------------------------------------------------
        # STATE FOR BUFFERING CLASSIFICATIONS
        #
        # - self.object_present == False  →  no object in view, buffer is empty.
        # - The moment YOLO first sees a box, set self.object_present = True and start
        #   appending (label, conf) each frame while there is at least one box.
        # - When YOLO reports zero boxes again (object left), pick the best (label,conf),
        #   schedule a timer to publish pusher after the appropriate delay, then clear buffer
        #   & reset self.object_present = False.
        self.object_present = False
        self.class_buffer = []  # will store tuples: (label_str, conf_float)

        # --- timing configuration ---
        # belt_speed in m/s (must match your Gazebo plugin's max_velocity scaling)
        self.declare_parameter('belt_speed', 1.6667)
        self.belt_speed = self.get_parameter('belt_speed').value

        # distances (m) from camera to each pusher
        self.distances = {
            1: 6.4,   # furthest
            2: 4.8,
            3: 3.2,
            4: 0.5,   # closest
        }
        # an “extra” empirically‐tuned offset (in seconds) for each pusher
        self.extra_delays = {
            1: -0.0,
            2: -0.0,
            3: -0.0,
            4: -0.25,
        }

        self.get_logger().info("WasteDetectorClassifier ready")

    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # --- DETECTION (YOLO) ---
        det_res = self.det_model.predict(
            source=img,
            imgsz=640,
            conf=0.25,
            verbose=False
        )[0]
        boxes = det_res.boxes

        # CASE A: NO BOXES DETECTED RIGHT NOW
        if boxes is None or len(boxes) == 0:
            # If we previously “had” an object in view, that object has now left.
            if self.object_present:
                # 1) PICK HIGHEST‐CONFIDENCE FROM BUFFER:
                best_label, best_conf = max(self.class_buffer, key=lambda tpl: tpl[1])
                pidx = self.class_to_pusher.get(best_label, 0)

                if pidx != 0:
                    # 2) COMPUTE DELAY = distance / belt_speed + extra_offset
                    dist = self.distances[pidx]
                    delay = dist / self.belt_speed + self.extra_delays.get(pidx, 0.0)

                    self.get_logger().info(
                        f"Object left → best='{best_label}' (conf={best_conf:.2f}) "
                        f"→ scheduling pusher {pidx} in {delay:.2f}s"
                    )

                    # 3) SCHEDULE EXACTLY ONE PUBLISH AFTER `delay` SECONDS:
                    def timer_cb():
                        # Publish exactly once, then cancel the timer
                        self.pusher_pub.publish(UInt8(data=pidx))
                        timer.cancel()

                    timer = self.create_timer(delay, timer_cb)

                else:
                    self.get_logger().info(
                        f"Object left → best='{best_label}' (conf={best_conf:.2f}) "
                        f"→ no pusher assigned"
                    )

                # 4) CLEAR BUFFER & RESET STATE
                self.class_buffer.clear()
                self.object_present = False

            # If we were not seeing an object, do nothing
            return

        # CASE B: AT LEAST ONE BOX DETECTED RIGHT NOW
        # If object_present was False, start buffering now:
        if not self.object_present:
            self.get_logger().info("New object detected → beginning to buffer classifications")
            self.object_present = True
            self.class_buffer.clear()

        # Pick the highest‐confidence YOLO detection bounding box this frame:
        confs = boxes.conf.cpu().numpy().flatten()
        best_idx = int(confs.argmax())
        x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[best_idx].astype(int)

        # Crop that region and run the classifier
        crop = img[y1:y2, x1:x2]
        cls_res = self.cls_model.predict(
            source=crop,
            imgsz=224,
            conf=0.0,
            verbose=False
        )[0]

        if cls_res.probs is None:
            self.get_logger().warn("Classification returned no probability scores—skipping frame.")
            return

        idx = int(cls_res.probs.top1)
        cls_conf = float(cls_res.probs.top1conf)
        label = cls_res.names[idx]

        # Append this frame’s result into the buffer
        self.class_buffer.append((label, cls_conf))
        # Don’t publish yet; wait for the object‐gone event.
        return


def main(args=None):
    rclpy.init(args=args)
    node = WasteDetectorClassifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
