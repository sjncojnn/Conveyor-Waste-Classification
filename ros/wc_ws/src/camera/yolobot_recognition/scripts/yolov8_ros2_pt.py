#!/usr/bin/env python3
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from datetime import datetime

def suppress_overlaps(boxes, iou_thresh=0.3):
    """
    Given boxes = det_res.boxes (Ultralytics Boxes object),
    return a list of indices such that no two kept boxes have IoU > iou_thresh.
    """
    if boxes is None or len(boxes) == 0:
        return []

    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy().flatten()
    idxs = np.argsort(-confs)

    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        x1_i, y1_i, x2_i, y2_i = xyxy[i]
        area_i = (x2_i - x1_i) * (y2_i - y1_i)

        rest_idxs = idxs[1:]
        new_rest = []
        for j in rest_idxs:
            x1_j, y1_j, x2_j, y2_j = xyxy[j]
            inter_x1 = max(x1_i, x1_j)
            inter_y1 = max(y1_i, y1_j)
            inter_x2 = min(x2_i, x2_j)
            inter_y2 = min(y2_i, y2_j)
            w = max(0.0, inter_x2 - inter_x1)
            h = max(0.0, inter_y2 - inter_y1)
            inter = w * h
            area_j = (x2_j - x1_j) * (y2_j - y1_j)
            union = area_i + area_j - inter
            iou = inter / union if union > 0 else 0
            if iou < iou_thresh:
                new_rest.append(j)
        idxs = np.array(new_rest, dtype=int)
    return keep

class WasteDetectorClassifier(Node):
    def __init__(self):
        super().__init__('waste_detector_classifier')
        self.bridge = CvBridge()

        # Directory for saving crops
        self.SAVE_DIR = os.path.expanduser('~/Conveyor-Waste-Classification/wc_ws/src/image')
        os.makedirs(self.SAVE_DIR, exist_ok=True)

        # Load YOLO weights
        share = get_package_share_directory('yolobot_recognition')
        det_weights = os.path.join(share, 'scripts', 'Detect_YOLOv8.pt')
        cls_weights = os.path.join(share, 'scripts', 'Classi_YOLOv8.pt')
        self.det_model = YOLO(det_weights)
        self.cls_model = YOLO(cls_weights)

        # Class → pusher index
        self.class_to_pusher = {
            'Plastic':            1,
            'Organic_Food_Waste': 2,
            'Paper_Cardboard':    3,
            'Metal_Other':        4,
            # 'Glass' → no pusher
        }

        # Subscriptions & publishers
        self.create_subscription(Image, 'rgb_cam/image_raw', self.camera_callback, 20)
        self.vis_pub = self.create_publisher(Image, 'inference_image', 20)
        self.pusher_pub = self.create_publisher(UInt8, '/pusher_index', 20)

        # Buffering state
        self.object_present = False
        self.class_buffer = []
        self.BUFFER_SIZE = 30
        self.no_box_count = 0
        self.NO_BOX_THRESHOLD = 2

        # Parameters
        self.declare_parameter('belt_speed', 1.6667)
        self.belt_speed = self.get_parameter('belt_speed').value

        self.declare_parameter('classify_delay', 0.25)
        self.classify_delay = self.get_parameter('classify_delay').value

        # Arm joint Y coordinates (meters), given camera at y=-3.5
        # Object leaves camera frame at y = -3.0
        # arm3      at y = -2.5
        # arm2      at y = -2.5 + 1.6 = -0.9
        # arm1      at y = -2.5 + 1.6*2 = 0.7
        # arm0      at y = -2.5 + 1.6*3 = 2.3
        self.arm_y = {  
            4: -2.5,  # arm3
            3: -0.9,  # arm2
            2:  0.7,  # arm1
            1:  2.3,  # arm0
        }

        # Y coordinate where object truly leaves camera view
        self.y_cam_exit = -4.6

        # Compute how far object drifts during classification (SIM meters)
        # Assuming sim runs in real time (RTF = 1.0)
        self.D_classged = self.belt_speed * self.classify_delay

        # Extra per-arm delay offsets (seconds, wall-clock)
        self.extra_delay = {
            4: -0.6,
            3: 0.75,
            2: 1.25,
            1: 2.0,
        }

        # For latency logging
        self.appear_time = None

        self.get_logger().info(f"WasteDetectorClassifier ready. Crops at: {self.SAVE_DIR}")

    def camera_callback(self, msg: Image):
        current_wall = time.time()

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        vis_img = img.copy()
        img_h, img_w = img.shape[:2]

        det_res = self.det_model.predict(source=img, imgsz=640, conf=0.25, verbose=False)[0]
        boxes = det_res.boxes

        # Draw detection boxes for debug
        if boxes is not None and len(boxes) > 0:
            bboxes  = boxes.xyxy.cpu().numpy()
            cls_ids = boxes.cls.cpu().numpy().astype(int)
            confs   = boxes.conf.cpu().numpy().flatten()
            for (x1, y1, x2, y2), cls_id, conf in zip(bboxes, cls_ids, confs):
                x1_i, y1_i, x2_i, y2_i = map(int, (x1, y1, x2, y2))
                label = det_res.names[cls_id]
                cv2.rectangle(vis_img, (x1_i, y1_i), (x2_i, y2_i), (0,255,0), 2)
                cv2.putText(vis_img, f"{label} {conf:.2f}", (x1_i, y1_i-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Publish annotated image
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, 'bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish vis image: {e}")

        # No detections → check for “object left”
        if boxes is None or len(boxes) == 0:
            if self.object_present:
                self.no_box_count += 1
                if self.no_box_count < self.NO_BOX_THRESHOLD:
                    return

                # Majority-vote
                counts = {}
                for (lbl, c) in self.class_buffer:
                    counts[lbl] = counts.get(lbl, 0) + 1

                best_label = None
                best_count = -1
                best_score = -1.0
                for lbl, cnt in counts.items():
                    avg_conf = sum(c for (l2, c) in self.class_buffer if l2 == lbl) / cnt
                    if cnt > best_count or (cnt == best_count and avg_conf > best_score):
                        best_label = lbl
                        best_count = cnt
                        best_score = avg_conf

                # Log classification latency
                classify_latency = current_wall - self.appear_time if self.appear_time else None
                if classify_latency is not None:
                    self.get_logger().info(
                        f"[LATENCY] Object '{best_label}' classification time = {classify_latency:.3f} s"
                    )

                pidx = self.class_to_pusher.get(best_label, 0)
                if pidx != 0:
                    D_cam_to_arm = abs(self.arm_y[pidx] - self.y_cam_exit)
                    D_remaining  = D_cam_to_arm - self.D_classged
                    if D_remaining <= 0.0:
                        wait_real = 0.0
                    else:
                        wait_real = D_remaining / self.belt_speed

                    # Apply extra offset (seconds)
                    wait_real += self.extra_delay.get(pidx, 0.0)

                    self.get_logger().info(
                        f"[OBJECT_LEFT] best='{best_label}' (avg_conf={best_score:.2f}, count={best_count}) → "
                        f"scheduling pusher {pidx} in {wait_real:.3f} s"
                    )

                    # Schedule pusher and log actual fire time
                    def timer_cb():
                        fire_time = time.time()
                        self.pusher_pub.publish(UInt8(data=pidx))
                        self.get_logger().info(
                            f"[LATENCY] Pusher {pidx} fired at +{(fire_time - current_wall):.3f} s "
                            f"(scheduled {wait_real:.3f} s)"
                        )
                        timer.cancel()

                    timer = self.create_timer(wait_real, timer_cb)

                else:
                    self.get_logger().info(
                        f"[OBJECT_LEFT] best='{best_label}' (count={best_count}) → no pusher"
                    )

                # Reset state
                self.class_buffer.clear()
                self.object_present = False
                self.no_box_count = 0

            return

        # At least one detection → start buffering if new object
        if not self.object_present:
            self.get_logger().info("[OBJECT_APPEARED] Buffering classifications")
            self.object_present = True
            self.class_buffer.clear()
            self.no_box_count = 0
            self.appear_time = time.time()  # record wall-clock time

        # NMS
        keep_idxs = suppress_overlaps(boxes, iou_thresh=0.3)

        # Filter out too-large boxes
        filtered_idxs = []
        for i in keep_idxs:
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            box_w = x2 - x1
            box_h = y2 - y1
            box_area = box_w * box_h
            frame_area = img_h * img_w
            if (box_area / float(frame_area)) < 0.30:
                filtered_idxs.append(i)

        if len(filtered_idxs) == 0:
            return

        # Choose highest-confidence
        best_idx = max(filtered_idxs, key=lambda i: boxes.conf[i])
        x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy().astype(int)

        # Crop and save
        crop = img[y1:y2, x1:x2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        temp_crop_path = os.path.join(self.SAVE_DIR, f"{timestamp}_rawcrop.png")
        cv2.imwrite(temp_crop_path, crop)

        # Classification
        cls_res = self.cls_model.predict(source=crop, imgsz=224, conf=0.0, verbose=False)[0]
        if cls_res.probs is None:
            self.get_logger().warn("Classification returned no probabilities—skipping frame")
            return

        idx = int(cls_res.probs.top1)
        cls_conf = float(cls_res.probs.top1conf)
        label = cls_res.names[idx]

        # Save labeled crop
        labeled_crop_path = os.path.join(
            self.SAVE_DIR,
            f"{timestamp}_{label}_{cls_conf:.2f}.png"
        )
        try:
            os.rename(temp_crop_path, labeled_crop_path)
        except Exception:
            cv2.imwrite(labeled_crop_path, crop)

        # Buffer
        self.class_buffer.append((label, cls_conf))
        if len(self.class_buffer) > self.BUFFER_SIZE:
            self.class_buffer.pop(0)

        self.get_logger().info(f"[CLASSIFY] {label} ({cls_conf:.2f}); buffer size = {len(self.class_buffer)}")

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
