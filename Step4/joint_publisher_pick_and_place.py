#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


def linear_interpolate(p1: np.ndarray, p2: np.ndarray, a: float) -> np.ndarray:
    """Return (1-a)*p1 + a*p2 for a in [0,1]."""
    return (1.0 - a) * p1 + a * p2


class JointPublisherPickAndPlace(Node):
    def __init__(self):
        super().__init__('joint_publisher_pick_and_place')

        # Publisher to /joint_states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # UR3e joint names MUST match the URDF exactly
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # Define start/end joint vectors (R^6)
        # You can change these to whatever your lab wants, but keep 6 values.
        self.p1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.p2 = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0], dtype=float)

        # Timing
        self.timer_period = 0.02  # 50 Hz
        self.t = 0.0

        # Timer callback
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('pick-and-place joint publisher started (publishing to /joint_states)')

    def timer_callback(self):
        # Decide interpolation parameter a based on time t in [0,20)
        if self.t < 5.0:
            a = 0.0
        elif self.t < 10.0:
            a = (self.t - 5.0) / 5.0          # 0 -> 1
        elif self.t < 15.0:
            a = 1.0
        else:
            a = 1.0 - (self.t - 15.0) / 5.0   # 1 -> 0

        # Interpolated joint position
        p = linear_interpolate(self.p1, self.p2, a).tolist()

        # Build JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = p
        # velocity/effort can be left empty for RViz visualization

        self.pub.publish(msg)

        # Optional: print occasionally so you know it's alive (every ~1 sec)
        if int(self.t / self.timer_period) % int(1.0 / self.timer_period) == 0:
            self.get_logger().info(f"t={self.t:0.2f}s a={a:0.2f} pos={['%0.2f'%x for x in p]}")

        # Update time, wrap every 20 seconds
        self.t += self.timer_period
        if self.t >= 20.0:
            self.t = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisherPickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()