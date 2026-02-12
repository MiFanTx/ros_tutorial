from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joint_listener and joint_commander together."""
    
    # TODO 1: Create a Node action for joint_listener
    # Hints:
    # - package='ur3e_basics'
    # - executable='joint_listener'
    # - output='screen' (so you see the logs)
    listener_node = Node(
        package='ur3e_basics',
        executable='joint_listener',
        output='screen'
    )
    
    # TODO 2: Create a Node action for joint_commander
    commander_node = Node(
        package='ur3e_basics',
        executable='joint_commander',
        output='screen'
    )
    
    # TODO 3: Return LaunchDescription with both nodes
    # Hint: LaunchDescription takes a list
    return LaunchDescription([
        listener_node,
        commander_node
    ])