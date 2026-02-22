#!/usr/bin/env python3
"""
Static Transform Broadcaster
Publishes the camera frame relative to the robot base
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticCameraTF (Node):

    def __init__(self):
        super().__init__('static_camera_tf')

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_transform()

        self.get_logger().info('Static Transformed published!')

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

    def publish_camera_transform(self):
        # Create the transform message
        t = TransformStamped()
        
        # TODO: Set the timestamp
        # Hint: t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = self.get_clock().now().to_msg()
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # --- Translation (position) ---
        # Camera is 0.5m in front, 0.3m left, 0.6m up
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.3
        t.transform.translation.z = 0.6
        
        # --- Rotation (orientation) ---
        # Camera looks down at 45 degrees (pitch = -pi/4)
        roll = 0
        pitch = -math.pi/4
        yaw = 0

        q = self.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)



    
def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraTF()

    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()