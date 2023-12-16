from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mono_color_segmentation',
            namespace='/intel_realsense_r200_depth',
            executable='mono_color_segmentation',
            name='mono_color_segmentation_node',
            output='screen',
            parameters=[{'target_color': [155, 155, 155]}]
        )
    ])
