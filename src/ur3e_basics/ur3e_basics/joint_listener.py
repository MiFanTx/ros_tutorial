#!/usr/bin/env python3
"""
Joint State Listener Node
Subscribes to /joint_states and prints shoulder position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointListener(Node):
    """
    Simple node that listens to robot joint states.
    
    Think of this like a YouTube subscriber:
    - You subscribe to a channel (/joint_states)
    - You get notifications (callbacks) when new videos (messages) arrive
    - You process the content (print joint positions)
    """
    
    def __init__(self):
        super().__init__('joint_listener')
        self.get_logger().info('Initializing Joint Listener Node...')
        
        # - MessageType is imported at top
        # - callback_name is the method below
        # - queue_size = 10 is standard

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        
        
        self.get_logger().info('Joint listener started! Waiting for joint states...')
    
    def joint_callback(self, msg):
        """
        Called every time a new JointState message arrives.
        
        msg structure:
        - msg.name: ['shoulder_pan_joint', 'shoulder_lift_joint', ...]
        - msg.position: [0.5, -1.2, 0.8, ...]  (radians)
        """
        
        # Steps:
        # 1. Loop through msg.name with enumerate to get index
        # 2. Check if joint_name == 'shoulder_pan_joint'
        # 3. Get position using that index: msg.position[i]
        # 4. Print using self.get_logger().info(f'...')

        for i, joint_name in enumerate(msg.name):
            if joint_name == 'shoulder_pan_joint':
                shoulder_position = msg.position[i]
                self.get_logger().info(f'Shoulder Pan Joint Position: {shoulder_position:.4f} radians')
                break
        


def main(args=None):
    """Main function to run the node."""    
    rclpy.init(args=args)
    node = JointListener()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()