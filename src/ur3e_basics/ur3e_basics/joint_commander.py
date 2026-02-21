#!/usr/bin/env python3
"""
Joint Commander Node
Publishes target joint positions for testing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger


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
        self.declare_parameter('publish_rate', 10.0)  # Publish every 0.1 seconds (10 Hz)
        self.declare_parameter('amplitude', 1.0)  # Amplitude of sine wave (radians)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value

        self.period = 1.0 / self.publish_rate  # Time between publishes
        
        self.create_service(Trigger, 'reset_counter', self.reset_callback)

        self.add_on_set_parameters_callback(self.parameter_callback)

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

    def parameter_callback(self, params):
        """
        Called when parameters are updated.
        Updates publish_rate and amplitude.
        """
        for param in params:
            if param.name =='amplitude':
                if param.value >= 2.0 or param.value <= 0.0:
                    return SetParametersResult(successful=False, reason='Amplitude must be between 0 and 2 radians')
                self.amplitude = param.value
                self.get_logger().info(f'Amplitude updated to: {self.amplitude}')
            elif param.name == 'publish_rate':
                self.get_logger().info(f'publish_rate change requires restart')
        
        return SetParametersResult(successful=True)
    
    def reset_callback(self, request, response):
        """
        Service callback to reset the counter.
        Resets the sine wave to start from 0 again.
        """
        self.counter = 0.0
        response.success = True
        response.message = 'Counter reset successfully'
        self.get_logger().info('Counter reset via service call!')
        return response


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