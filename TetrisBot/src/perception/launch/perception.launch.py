from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RealSense (include rs_launch.py)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'rgb_camera.color_profile': '1920x1080x30',
        }.items(),
    )

    # ArUco recognition
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_aruco'),
                'launch',
                'aruco_recognition.launch.py'
            )
        )
    )
    ar_marker_launch_arg = DeclareLaunchArgument(
        'ar_marker',
        # default_value='ar_marker_7'
        default_value='9'
    )
    ar_marker= LaunchConfiguration('ar_marker')

    ar_tag_identification_node = Node(
        package='perception',
        executable='ar_tag_identify',
        name='ar_tag_identification_node',
        parameters=[
            {
                'base_marker': ar_marker,
            }
        ],
        output='screen',
    )

    # Static TF: base_link -> world
    # -------------------------------------------------
    # This TF is static because the "world" frame does not move.
    # It is necessary to define the "world" frame for MoveIt to work properly as this is the defualt planning frame.
    # -------------------------------------------------
    static_base_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_world',
        arguments=['0','0','0','0','0','0','1','base_link','world'],
        output='screen',
    )

    # ar_marker_launch_arg = DeclareLaunchArgument(
    #     'ar_marker',
    #     default_value='ar_marker_8'
    # )
    # ar_marker = LaunchConfiguration('ar_marker')

    # # Planning TF node
    # planning_tf_node = Node(
    #     package='planning',
    #     executable='tf',
    #     name='tf_node',
    #     output='screen',
    #     parameters=[{
    #         'ar_marker': ar_marker,
    #     }]
    # )
    # # Planning TF node launch
    # tf_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('planning'),
    #             'launch',
    #             'tf.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'ar_marker': ar_marker
    #     }.items(),
    # )

    # MoveIt 
    ur_type = LaunchConfiguration("ur_type", default="ur7e")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    # Path to the MoveIt launch file
    moveit_launch_file = os.path.join(
                get_package_share_directory("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            )

    # Include the MoveIt launch description
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz
        }.items(),
    )

    ik_planner_node = Node(
        package='planning',
        executable='ik',
        name='ik_planner',
        output='screen',
        parameters=[
            {'base_frame': 'base_link'},
            {'group_name': 'ur_manipulator'},
            {'ik_link_name': 'tool0'},      # NOTE might have to change if EE link differs
            {'ik_timeout_sec': 2.0},
            {'planning_time_sec': 5.0},
            {'joint_tolerance': 0.01},
        ],
    )

    # -------------------------
    # Global shutdown on any process exit
    # -------------------------
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason='SOMETHING BONKED'))]
        )
    )

    # return LaunchDescription([
    #     ar_tag_identification_node,
    # ])

    return LaunchDescription([
        ar_marker_launch_arg,
        realsense_launch,
        aruco_launch,
        static_base_world,
        # planning_tf_node,
        # tf_launch,
        moveit_launch,
        ar_tag_identification_node,
        ik_planner_node,
        shutdown_on_any_exit
    ])
