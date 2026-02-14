from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch joint_listener and joint_commander together."""
    
    # TODO 1: Create a Node action for joint_listener
    # Hints:
    # - package='ur3e_basics'
    # - executable='joint_listener'
    # - output='screen' (so you see the logs)

    config = os.path.join(
        get_package_share_directory('ur3e_basics'),
        'config', 
        'robot_params.yaml'
    )

    listener_node = Node(
        package='ur3e_basics',
        executable='joint_listener',
        output='screen'
    )
    
    # TODO 2: Create a Node action for joint_commander
    commander_node = Node(
        package='ur3e_basics',
        executable='joint_commander',
        output='screen',
        parameters=[config] # Load parameters from YAML file
    )
    
    # TODO 3: Return LaunchDescription with both nodes
    # Hint: LaunchDescription takes a list
    return LaunchDescription([
        listener_node,
        commander_node
    ])