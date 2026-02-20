#!/usr/bin/env python3
"""
Pick and Place Action Server
Simulates a pick-and-place operation with feedback
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from ur3e_basics.action import PickPlace


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        
        # TODO 1: Create the action server
        # Hint: ActionServer(self, ActionType, 'action_name', callback)
        self.action_server = None  # Replace
        
        self.get_logger().info('Pick and Place Server ready!')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the pick-and-place operation."""
        self.get_logger().info(f'Received goal: Pick "{goal_handle.request.object_id}"')
        
        # TODO 2: Create a feedback message
        feedback = None  # Replace
        
        stages = [
            ('MOVING_TO_OBJECT', 20.0, 'Moving to pick position...'),
            ('LOWERING', 35.0, 'Lowering to object...'),
            ('GRASPING', 50.0, 'Closing gripper...'),
            ('LIFTING', 65.0, 'Lifting object...'),
            ('MOVING_TO_TARGET', 80.0, 'Moving to place position...'),
            ('PLACING', 90.0, 'Opening gripper...'),
            ('RETREATING', 100.0, 'Retreating to safe position...'),
        ]
        
        start_time = time.time()
        
        for stage_name, progress, status_msg in stages:
            # TODO 3: Update feedback fields
            
            # TODO 4: Publish the feedback
            
            self.get_logger().info(f'  [{progress:.0f}%] {stage_name}: {status_msg}')
            time.sleep(1.0)
        
        # TODO 5: Create and populate the result
        result = None  # Replace
        
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()