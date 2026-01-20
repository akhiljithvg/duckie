#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import time


class VisionControl(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ---------- ROS ----------
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_cb,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_motor',
            10
        )

        # ---------- IMAGE ----------
        self.W = 320
        self.H = 240

        # ---------- ARUCO (PI SAFE OLD API) ----------
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.ARUCO_AREA_THRESHOLD = 2000
        self.ARUCO_COOLDOWN = 2.0
        self.last_aruco_time = 0.0

        # ---------- CONTROL ----------
        self.BASE_SPEED = 15.0

        self.get_logger().info("✅ FULL corrected perception node started")

    # ------------------------------------------------
    def publish_cmd(self, speed, steer):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steer)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ================= ARUCO DETECTION =================
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        now = time.time()

        if ids is not None:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])

                # ---- FIXED CONTOUR SHAPE ----
                pts = corners[i][0].astype(int)   # (4,2)
                area = int(cv2.contourArea(pts))

                # ---- DEBUG OVERLAY ----
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                cx, cy = pts.mean(axis=0).astype(int)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    f"ID:{marker_id} A:{area}",
                    (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # ---- LOG + COOLDOWN ----
                if area > self.ARUCO_AREA_THRESHOLD and \
                   (now - self.last_aruco_time) > self.ARUCO_COOLDOWN:

                    self.last_aruco_time = now
                    self.get_logger().info(
                        f"Detected ArUco ID={marker_id}, area={area}"
                    )

                    # ---- PLACEHOLDER ACTIONS ----
                    # (NON-BLOCKING – SAFE)
                    if marker_id == 5:
                        self.publish_cmd(self.BASE_SPEED, 0.0)

                    elif marker_id == 7:
                        self.publish_cmd(0.0, 25.0)

                    elif marker_id == 4:
                        self.publish_cmd(0.0, -25.0)

                    elif marker_id == 6:
                        self.publish_cmd(0.0, 0.0)

        # ================= DISPLAY =================
        cv2.imshow("Perception Debug (ArUco)", frame)
        cv2.waitKey(1)

    # ------------------------------------------------
    def destroy_node(self):
        self.publish_cmd(0.0, 0.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
