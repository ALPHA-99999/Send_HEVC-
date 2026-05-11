from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="send_lb_capture_preprocess",
            executable="capture_preprocess_node",
            name="capture_preprocess_node",
            output="screen",
        ),
    ])
