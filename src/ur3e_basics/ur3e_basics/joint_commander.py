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

        self.publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # TODO 1: Declare parameters
        # Hint: self.declare_parameter('name', default_value)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('amplitude', 1.0)
        
        # TODO 2: Get parameter values
        # Hint: self.get_parameter('name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value
        
        # TODO 3: Use publish_rate to calculate timer period
        # Hint: period = 1.0 / rate
        self.period = 1.0 / self.publish_rate


        # TODO 4: Store amplitude for use in callback
        self.timer = self.create_timer(self.period, self.timer_callback)
        
        self.counter = 0.0  # For generating sine wave
        self.get_logger().info('Joint commander started! Publishing commands...')
    
    def timer_callback(self):
        """
        Called every 0.1 seconds by the timer.
        Generates and publishes a sine wave command.
        """
        
        # Create message
        msg = JointState()
        
        msg.name = ['shoulder_pan_joint']
        msg.position = [self.amplitude * math.sin(self.counter)]
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