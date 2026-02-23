#!/usr/bin/env python3
"""
Detection Transformer
Transforms detections from camera frame to robot base frame
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

from ur3e_basics.msg import Detection


class DetectionTransformer(Node):
    def __init__(self):
        super().__init__('detection_transformer')
        
        # TODO: Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # TODO: Subscribe to /detections

        self.sub = self.create_subscription(
            Detection,
            '/detections',
            self.detection_callback,
            10
        )

        # TODO: Publish to /detections_base_frame

        self.pub = self.create_publisher(
            Detection,
            '/detections_base_frame',
            10
        )
        
        self.get_logger().info('Detection Transformer started!')
    
    def detection_callback(self, msg: Detection):
        try:
            # TODO: Lookup transform from camera_link to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()
            )
            # TODO: Transform msg.pose (it's a PoseStamped)
            # Hint: transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(
            #           msg.pose, transform)
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                msg.pose.pose, transform)
            
            # TODO: Create new Detection message with transformed pose
            d = Detection()

            now = self.get_clock().now().to_msg()
            d.header.stamp = now

            d.header.frame_id = 'base_link'
            d.object_id = msg.object_id
            d.object_class = msg.object_class
            d.confidence = msg.confidence

            d.x_min = msg.x_min
            d.y_min = msg.y_min
            d.x_max = msg.x_max
            d.y_max = msg.y_max

            d.pose.header.stamp = now
            d.pose.header.frame_id = 'base_link'

            d.pose.pose = transformed_pose
            
            # TODO: Publish transformed detection
            self.pub.publish(d)

            # TODO: Log the transformation
            self.get_logger().info(
                f'{msg.object_id}: camera({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f}) → '
                f'base({d.pose.pose.position.x:.2f}, {d.pose.pose.position.y:.2f}, {d.pose.pose.position.z:.2f})'
            )

        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTransformer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()