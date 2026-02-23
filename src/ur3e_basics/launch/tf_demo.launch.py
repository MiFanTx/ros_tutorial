from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    static_camera_tf = Node(
        package='ur3e_basics',
        executable='static_camera_tf.py',
        output='screen'
    )

    rotating_platform_tf = Node(
        package='ur3e_basics',
        executable='rotating_platform_tf.py',
        output='screen'
    )

    tf_listener = Node(
        package='ur3e_basics',
        executable='tf_listener.py',
        output='screen'
    )
    
    return LaunchDescription([
        static_camera_tf,
        rotating_platform_tf,
        tf_listener
    ])