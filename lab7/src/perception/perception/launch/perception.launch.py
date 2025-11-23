from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ar_tag_identification_node = Node(
        package='perception',
        executable='ar_tag_identification',
        name='ar_tag_identification_node',
        parameters=[
            {
                'base_marker': 7,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        ar_tag_identification_node,
    ])
