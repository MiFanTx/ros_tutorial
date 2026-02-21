#!/usr/bin/env python3
"""
Detection Filter Node
Filters detections by confidence threshold
"""

import rclpy
from rclpy.node import Node

from ur3e_basics.msg import Detection


class DetectionFilter(Node):
    def __init__(self):
        super().__init__('detection_filter')
        
        # TODO: Declare parameter 'min_confidence' with default 0.9
        self.declare_parameter('min_confidence', 0.9)
        # TODO: Create subscriber to '/detections'
        self.sub = self.create_subscription(Detection, '/detections', self.detection_callback, 10)
        # TODO: Create publisher to '/filtered_detections'
        self.pub = self.create_publisher(Detection, '/filtered_detections', 10)
        
        self.get_logger().info('Detection Filter started!')
    
    def detection_callback(self, msg: Detection):
        # TODO: Get current min_confidence parameter value
        min_conf = self.get_parameter('min_confidence').value
        # TODO: Check if detection meets threshold
        if msg.confidence >= float(min_conf):
            self.pub.publish(msg)
            self.get_logger().info(
                f"PASS {msg.object_id} conf={msg.confidence:.2f} >= {float(min_conf):.2f}"
            )
        else:
            self.get_logger().info(
                f"DROP {msg.object_id} conf={msg.confidence:.2f} < {float(min_conf):.2f}"
            )
        # TODO: If yes, republish to filtered topic and log
        
        # TODO: If no, log that it was filtered out
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DetectionFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()