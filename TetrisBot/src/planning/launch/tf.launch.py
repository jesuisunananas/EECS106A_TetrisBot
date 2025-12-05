import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    ar_marker_launch_arg = DeclareLaunchArgument(
        'ar_marker',
        default_value='7'
    )
    ar_marker = LaunchConfiguration('ar_marker')

    # Planning TF node
    planning_tf_node = Node(
        package='planning',
        executable='tf',
        name='tf_node',
        output='screen',
        parameters=[{
            'ar_marker': ar_marker,
        }]
    )

    return LaunchDescription([
        ar_marker_launch_arg,
        planning_tf_node
    ])
