#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from ultralytics import YOLO

import cv2
import numpy as np
import math
import json


class CameraErkennung(Node):

    def __init__(self):
        super().__init__("camera_erkennung")

        # =========================
        # YOLO
        # =========================
        self.model = YOLO("yolov8n.pt")

        # =========================
        # CAMERA PARAMETER
        # =========================
        self.image_width = 640
        self.fov = math.radians(60)

        # MITTE-TOLERANZ
        self.center_threshold = math.radians(10)

        # =========================
        # LIDAR PARAMETER
        # =========================
        self.lidar_front_angle = 20

        self.latest_scan = None

        # =========================
        # PUBLISHER
        # =========================
        self.pub = self.create_publisher(
            String,
            "/robot/stopsign",
            10
        )

        self.pub_image = self.create_publisher(
            CompressedImage,
            "/robot/camera/processed",
            10
        )

        # =========================
        # SUBSCRIBER
        # =========================
        # Best-Effort QoS (Sensor-Profil): Verhindert Netzwerk-Lags über WLAN durch Verwerfen alter Pakete
        self.create_subscription(
            CompressedImage,
            "/depth_cam/rgb/image_raw/compressed",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("Camera + LiDAR Fusion gestartet")

    # =========================================================
    # LIDAR CALLBACK
    # =========================================================
    def lidar_callback(self, msg):
        self.latest_scan = msg

    def process_lidar(self, msg):

        ranges = list(msg.ranges)
        n = len(ranges)

        if n == 0:
            return None

        angle_step = msg.angle_increment
        front_range = int(math.radians(self.lidar_front_angle) / angle_step)

        center = n // 2

        front = ranges[center - front_range : center + front_range]
        left = ranges[0 : center - front_range]
        right = ranges[center + front_range :]

        def min_valid(data):
            valid = [
                d for d in data
                if math.isfinite(d)
                and d > msg.range_min
                and d < msg.range_max
            ]
            return min(valid) if valid else float("inf")

        # =========================
        # LIDAR AUSGABE (WICHTIG)
        # =========================
        self.get_logger().debug(
            f"[LIDAR] left={min_valid(right):.2f}m "
            f"front={min_valid(front):.2f}m "
            f"right={min_valid(left):.2f}m"
        )

        return {
            "front": min_valid(front),
            "left": min_valid(right),
            "right": min_valid(left)
        }

    # =========================================================
    # CAMERA CALLBACK
    # =========================================================
    def image_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame, verbose=False)[0]

        lidar = self.latest_scan
        if lidar is None:
            return

        lidar = self.process_lidar(lidar)
        if lidar is None:
            return

        stop_detected = False
        distance = None

        for box in results.boxes:

            cls_id = int(box.cls[0])
            name = self.model.names[cls_id]

            if name != "stop sign":
                continue

            stop_detected = True

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x_center = (x1 + x2) / 2

            # =========================
            # WINKEL
            # =========================
            angle = (x_center - self.image_width / 2) * (self.fov / self.image_width)

            # =========================
            # RICHTUNG MIT TOLERANZ
            # =========================
            if angle < -self.center_threshold:
                direction = "left"
            elif angle > self.center_threshold:
                direction = "right"
            else:
                direction = "front"

            # =========================
            # CAMERA AUSGABE
            # =========================
            self.get_logger().info(
                f"[CAMERA] stop sign → {direction} | angle={math.degrees(angle):.2f}°"
            )

            # =========================
            # LIDAR MATCHING (SAFE)
            # =========================
            distance = lidar.get(direction, float("inf"))

            # =========================
            # FUSION AUSGABE
            # =========================
            if distance is not None:
                self.get_logger().info(
                    f"[FUSION] dir={direction} | dist={distance:.2f} m"
                )
            else:
              self.get_logger().info(
                 f"[FUSION] dir={direction} | dist=unbekannt"
                )

            # =========================
            # VISUAL
            # =========================
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if distance is not None:
                label = f"{direction} {distance:.2f}m"
            else:
                 label = direction

            cv2.putText(
                frame,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        if not stop_detected:
            self.get_logger().debug("[CAMERA] kein Stoppschild erkannt")
            
        # =========================
        # FUSION MESSAGE
        # =========================
        fusion_msg = String()

        fusion_msg.data = json.dumps({
            "detected": stop_detected,
            "distance": distance
        })

        self.pub.publish(fusion_msg)

        # =========================
        # IMAGE OUTPUT
        # =========================
        success, buffer = cv2.imencode(".jpg", frame)

        if success:
            img_msg = CompressedImage()
            img_msg.format = "jpeg"
            img_msg.data = buffer.tobytes()
            self.pub_image.publish(img_msg)

        cv2.imshow("Fusion Camera", frame)
        cv2.waitKey(1)


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()
    node = CameraErkennung()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()