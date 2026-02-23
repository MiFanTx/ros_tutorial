from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    static_camera_tf = Node(
        package='ur3e_basics',
        executable='static_camera_tf.py',
        output='screen'
    )
    detection_publisher = Node(
        package='ur3e_basics',
        executable='detection_publisher.py',
        output='screen'
    )
    detection_transformer = Node(
        package='ur3e_basics',
        executable='detection_transformer.py',
        output='screen'
    )
    detection_filter = Node(
        package='ur3e_basics',
        executable='detection_filter.py',
        parameters=[{'min_confidence': 0.90}],
        remappings= [('/detections','/detections_base_frame')],
        output='screen'
    )

    return LaunchDescription([
        static_camera_tf,
        detection_publisher,
        detection_transformer,
        detection_filter
    ])