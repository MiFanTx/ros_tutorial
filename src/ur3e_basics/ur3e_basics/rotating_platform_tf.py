#!/usr/bin/env python3
"""
Dynamic Transform Broadcaster
Simulates a rotating sensor platform
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time


class RotatingPlatformTF(Node):
    def __init__(self):
        super().__init__('rotating_platform_tf')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create timer for 50Hz updates (period = 0.02 seconds)
        self.peroid = 0.02
        self.timer = self.create_timer(self.peroid, self.timer_callback)   

        # Record start time for calculating rotation angle
        self.start_time = time.time()
        
        self.get_logger().info('Rotating platform TF started!')
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (in radians) to quaternion.
        
        Args:
            roll: Rotation around X axis
            pitch: Rotation around Y axis  
            yaw: Rotation around Z axis
        
        Returns:
            List [w, x, y, z] quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [w, x, y, z]
    
    def timer_callback(self):
        # Calculate yaw angle based on elapsed time
        elapsed = time.time() - self.start_time
        rotation_period = 10.0  # seconds per full rotation
        yaw = (elapsed / rotation_period) * 2 * math.pi
        
        # Create TransformStamped message
        t = TransformStamped()
        
        # Set header.stamp, frame_id, child_frame_id
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rotating_platform'

        # Set translation (0.2, 0, 0.1)
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        # Set rotation using quaternion_from_euler(0, 0, yaw)
        q = self.quaternion_from_euler(0, 0, yaw)        
        # Broadcast the transform
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RotatingPlatformTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()