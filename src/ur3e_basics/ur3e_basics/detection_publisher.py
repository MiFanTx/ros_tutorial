#!/usr/bin/env python3
"""
Detection Publisher Node
Simulates object detection and publishes Detection messages
"""

import math
import random

import rclpy
from rclpy.node import Node

from ur3e_basics.msg import Detection


class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')

        # TODO: publisher to '/detections' (queue size 10)
        self.pub = self.create_publisher(Detection, '/detections', 10)

        # TODO: timer at 1.0 Hz -> self.timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)


        # Define 3 simulated objects with base pose + bbox
        # Convention: bbox pixels are x_min, y_min, x_max, y_max
        self.objects = [
            # TODO: fill these three dicts
            {
              'id': 'red_cube',
              'class': 'cube',
              'confidence': 0.95,
              'base_xyz': (0.2, 0.3, 0.5),
              'bbox': (100, 120, 180, 200),
            },
            {
              'id': 'blue_cylinder',
              'class': 'cylinder',
              'confidence': 0.87,
              'base_xyz': (0.30, 0.10, 0.50),
              'bbox': (400, 420, 480, 520),
            },
            {
              'id': 'green_sphere',
              'class': 'sphere',
              'confidence': 0.92,
              'base_xyz': (0.80, 0.80, 0.80),
              'bbox': (100, 120, 680, 600),
            },
        ]

        self.get_logger().info('Detection Publisher started!')

    def add_noise(self, value: float, noise_amount: float = 0.01) -> float:
        return value + random.uniform(-noise_amount, noise_amount)

    def timer_callback(self):

        obj = random.choice(self.objects)

        msg = Detection()

        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "camera_frame"

        msg.object_id = obj['id']
        msg.object_class = obj['class']
        msg.confidence = float(obj['confidence'])


        x_min, y_min, x_max, y_max = obj["bbox"]
        msg.x_min = int(x_min)
        msg.y_min = int(y_min)
        msg.x_max = int(x_max)
        msg.y_max = int(y_max)

        msg.pose.header.stamp = now
        msg.pose.header.frame_id = "camera_frame"

        bx, by, bz = obj['base_xyz']
        msg.pose.pose.position.x = self.add_noise(bx, 0.01)
        msg.pose.pose.position.y = self.add_noise(by, 0.01)
        msg.pose.pose.position.z = self.add_noise(bz, 0.01)

        msg.pose.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info(
            f"Detected {msg.object_id} ({msg.object_class}) conf={msg.confidence:.2f} "
            f"pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()