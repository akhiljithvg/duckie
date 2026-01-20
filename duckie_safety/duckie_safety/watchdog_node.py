#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MotorWatchdog(Node):
    def __init__(self):
        super().__init__('motor_watchdog')

        # -------- Parameters --------
        self.TIMEOUT = 0.5  # seconds without cmd -> STOP

        # -------- State --------
        self.last_cmd_time = time.time()
        self.last_cmd = Twist()

        # -------- ROS --------
        self.sub = self.create_subscription(
            Twist,
            '/cmd_motor_raw',
            self.cmd_cb,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_motor',
            10
        )

        # Timer checks safety
        self.timer = self.create_timer(0.05, self.check_timeout)

        self.get_logger().info("🛡️ Motor safety watchdog started")

    # ----------------------------------
    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = time.time()
        self.last_cmd = msg
        self.pub.publish(msg)

    # ----------------------------------
    def check_timeout(self):
        if time.time() - self.last_cmd_time > self.TIMEOUT:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.pub.publish(stop)

    # ----------------------------------
    def destroy_node(self):
        stop = Twist()
        self.pub.publish(stop)
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
