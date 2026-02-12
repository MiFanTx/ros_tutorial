#!/usr/bin/env python3
"""
Joint Commander Node
Publishes target joint positions for testing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointCommander(Node):
    """
    Publishes sine wave joint commands for testing.
    
    Think of this as a test signal generator:
    - Generates smooth back-and-forth motion
    - Helps test if subscribers are working
    - No real robot control (just demonstration)
    """
    
    def __init__(self):
        super().__init__('joint_commander')
        
        # TODO 1: Create publisher on '/joint_commands' topic
        # Hint: self.create_publisher(MessageType, '/topic_name', queue_size)
        # Use JointState message, queue_size = 10

        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)
        
        
        # TODO 2: Create timer to publish at 10 Hz (every 0.1 seconds)
        # Hint: self.create_timer(timer_period_sec, callback_function)
        # timer_period = 0.1 for 10 Hz
        # callback = self.timer_callback
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.counter = 0.0  # For generating sine wave
        self.get_logger().info('Joint commander started! Publishing commands...')
    
    def timer_callback(self):
        """
        Called every 0.1 seconds by the timer.
        Generates and publishes a sine wave command.
        """
        
        # Create message
        msg = JointState()
        
        # TODO 3: Fill in the message fields
        # msg.name = ['shoulder_pan_joint']  # Joint name
        # msg.position = [value]  # Target position in radians
        # 
        # Generate sine wave: amplitude * sin(counter)
        # amplitude = 1.0 (move ±1 radian)
        # Use: math.sin(self.counter)
        # Increment counter: self.counter += 0.1
        
        msg.name = ['shoulder_pan_joint']
        msg.position = [1.0 * math.sin(self.counter)]
        self.counter += 0.1

        
        # Publish
        self.publisher.publish(msg)

        # Log occasionally (not every message - too much!)
        if int(self.counter * 10) % 20 == 0:  # Every 2 seconds
            self.get_logger().info(f'Command: {msg.position[0]:.2f} rad')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()