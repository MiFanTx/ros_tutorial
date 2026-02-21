from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detection_publisher = Node(
        package='ur3e_basics',
        executable='detection_publisher.py',
        output='screen'
    )

    detection_filter = Node(
        package='ur3e_basics',
        executable='detection_filter.py',
        parameters=[{'min_confidence': 0.85}],
        output='screen'
    )
    
    return LaunchDescription([
        # TODO: Add detection_publisher node
        detection_publisher,
        # TODO: Add detection_filter node with min_confidence parameter
        detection_filter
    ])