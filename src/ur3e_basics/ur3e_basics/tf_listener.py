#!/usr/bin/env python3
"""
Transform Listener
Demonstrates looking up transforms and converting points
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # Required for do_transform_point


class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # TODO: Create TF buffer (stores transforms)
        # Hint: self.tf_buffer = Buffer()
        self.tf_buffer = Buffer()
        
        # TODO: Create TF listener (fills the buffer)
        # Hint: self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TODO: Create timer to periodically lookup transforms (1 Hz)
        self.timer = self.create_timer(1, self.timer_callback)
        
        # Define a test point in camera frame
        # (simulates object detected 30cm in front of camera)
        self.test_point = PointStamped()
        self.test_point.header.frame_id = 'camera_link'
        self.test_point.point.x = 0.0   # Directly in front
        self.test_point.point.y = 0.0   # Centered
        self.test_point.point.z = 0.3   # 30cm away from camera
        
        self.get_logger().info('TF Listener started!')
    
    def timer_callback(self):
        # Update timestamp to current time
        self.test_point.header.stamp = self.get_clock().now().to_msg()
        
        try:
            # TODO: Lookup transform from camera_link to base_link
            # Hint: transform = self.tf_buffer.lookup_transform(
            #           'base_link',           # Target frame
            #           'camera_link',         # Source frame  
            #           rclpy.time.Time()      # Latest available
            #       )

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()
            )
            
            # TODO: Transform the point
            # Hint: transformed_point = tf2_geometry_msgs.do_transform_point(
            #           self.test_point, transform)

            transformed_point = tf2_geometry_msgs.do_transform_point(
                self.test_point, transform)
            
            # TODO: Log both original and transformed points
            # self.get_logger().info(
            #     f'Point in camera_link: ({self.test_point.point.x:.2f}, ...)'
            #     f' → base_link: ({transformed_point.point.x:.2f}, ...)'
            # )
            pass
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()