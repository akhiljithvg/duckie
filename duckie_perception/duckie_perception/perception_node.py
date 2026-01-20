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
from enum import Enum


# ================= STATES =================
class RobotState(Enum):
    FOLLOW_LANE = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    GO_STRAIGHT = 3
    STOP = 4


class VisionControl(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ---------- ROS ----------
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_cb, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_motor', 10
        )

        # ---------- IMAGE ----------
        self.W = 320
        self.H = 240

        # ---------- LANE PARAMS ----------
        self.lower_red1 = np.array([0, 110, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 110, 70])
        self.upper_red2 = np.array([180, 255, 255])
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )

        self.BASE_SPEED = 15.0
        self.STEER_GAIN_POS = 0.10
        self.STEER_GAIN_ANG = 0.10
        self.MAX_STEER = 28.0
        self.MIN_PIXELS = 25

        # ---------- ARUCO ----------
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.ARUCO_AREA_THRESHOLD = 2000
        self.ARUCO_COOLDOWN = 2.0
        self.last_aruco_time = 0.0

        # ---------- STATE MACHINE ----------
        self.state = RobotState.FOLLOW_LANE
        self.state_start_time = time.time()

        self.TURN_DURATION = 1.0
        self.STRAIGHT_DURATION = 0.5
        self.STOP_DURATION = 2.0

        self.TURN_SPEED = 25.0

        self.get_logger().info("✅ Lane + ArUco state machine started")

    # ------------------------------------------------
    def publish_cmd(self, speed, steer):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steer)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------
    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"🔁 State → {new_state.name}")

    # ------------------------------------------------
    def follow_lane(self, frame, mask):
        # ---------- TOP BAND ----------
        y_top_low  = int(self.H * 0.05)
        y_top_high = int(self.H * 0.20)
        top_band = mask[y_top_low:y_top_high, :]
        col_top = np.sum(top_band // 255, axis=0)

        has_top = col_top.max() > self.MIN_PIXELS
        x_top = np.argmax(col_top) if has_top else None

        # ---------- MID BAND ----------
        y_mid_low  = int(self.H * 0.65)
        y_mid_high = int(self.H * 0.80)
        mid_band = mask[y_mid_low:y_mid_high, :]
        col_mid = np.sum(mid_band // 255, axis=0)

        has_mid = col_mid.max() > self.MIN_PIXELS
        x_mid = np.argmax(col_mid) if has_mid else None

        if not has_mid and not has_top:
            self.publish_cmd(self.BASE_SPEED, 0.0)
            return

        if not has_mid:
            x_mid = x_top
        if not has_top:
            x_top = x_mid

        pos_err = x_mid - self.W // 2
        ang_err = x_top - x_mid

        steer = (
            self.STEER_GAIN_POS * pos_err +
            self.STEER_GAIN_ANG * ang_err
        )

        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))

        self.publish_cmd(self.BASE_SPEED, steer)

        # ---------- DEBUG ----------
        cv2.circle(frame, (x_mid, y_mid_high), 6, (0,255,255), -1)
        cv2.circle(frame, (x_top, y_top_high), 6, (255,0,0), -1)
        cv2.line(
            frame,
            (self.W//2, self.H),
            (x_mid, y_mid_high),
            (0,255,0),
            2
        )

    # ------------------------------------------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now = time.time()

        # ================= ARUCO =================
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                pts = corners[i][0].astype(int)
                area = int(cv2.contourArea(pts))

                cv2.polylines(frame, [pts], True, (0,255,0), 2)
                cx, cy = pts.mean(axis=0).astype(int)
                cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)
                cv2.putText(
                    frame,
                    f"ID:{marker_id}",
                    (pts[0][0], pts[0][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2
                )

                if (
                    area > self.ARUCO_AREA_THRESHOLD and
                    (now - self.last_aruco_time) > self.ARUCO_COOLDOWN and
                    self.state == RobotState.FOLLOW_LANE
                ):
                    self.last_aruco_time = now

                    if marker_id == 7:
                        self.set_state(RobotState.TURN_LEFT)
                    elif marker_id == 4:
                        self.set_state(RobotState.TURN_RIGHT)
                    elif marker_id == 5:
                        self.set_state(RobotState.GO_STRAIGHT)
                    elif marker_id == 6:
                        self.set_state(RobotState.STOP)

        # ================= STATE HANDLER =================
        elapsed = now - self.state_start_time

        if self.state == RobotState.FOLLOW_LANE:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            m2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(m1, m2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, 1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, 2)

            self.follow_lane(frame, mask)

        elif self.state == RobotState.TURN_LEFT:
            self.publish_cmd(0.0, self.TURN_SPEED)
            if elapsed > self.TURN_DURATION:
                self.set_state(RobotState.FOLLOW_LANE)

        elif self.state == RobotState.TURN_RIGHT:
            self.publish_cmd(0.0, -self.TURN_SPEED)
            if elapsed > self.TURN_DURATION:
                self.set_state(RobotState.FOLLOW_LANE)

        elif self.state == RobotState.GO_STRAIGHT:
            self.publish_cmd(self.BASE_SPEED, 0.0)
            if elapsed > self.STRAIGHT_DURATION:
                self.set_state(RobotState.FOLLOW_LANE)

        elif self.state == RobotState.STOP:
            self.publish_cmd(0.0, 0.0)
            if elapsed > self.STOP_DURATION:
                self.set_state(RobotState.FOLLOW_LANE)

        # ================= DISPLAY =================
        cv2.imshow("Lane + ArUco State Machine", frame)
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
