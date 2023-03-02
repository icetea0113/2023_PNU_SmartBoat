import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # # detect 노드
    # mechaship_detect_params = LaunchConfiguration(
    #     "mechaship_detect_params",
    #     default=os.path.join(
    #         get_package_share_directory("mechaship_example"),
    #         "param",
    #         "mechaship_detect.yaml",
    #     ),
    # )
    # mechaship_detect_arg = DeclareLaunchArgument(
    #     "mechaship_detect_params",
    #     default_value=mechaship_detect_params,
    # )
    # mechaship_detect = Node(
    #     package="mechaship_example",
    #     executable="mechaship_detect_node",
    #     name="mechaship_detect_node",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[mechaship_detect_params],
    # )

    # classify 노드
    classify_params = LaunchConfiguration(
        "classify_params",
        default=os.path.join(
            get_package_share_directory("pnu2023"),
            "param",
            "classify.yaml",
        ),
    )
    mechaship_classify_arg = DeclareLaunchArgument(
        "classify_params",
        default_value=classify_params,
    )
    mechaship_classify = Node(
        package="pnu2023",
        executable="lidar_convert",
        name="lidar_convert",
        output="screen",
        emulate_tty=True,
        parameters=[classify_params],
    )

    # # navigation 노드
    # mechaship_navigation_params = LaunchConfiguration(
    #     "mechaship_navigation_params",
    #     default=os.path.join(
    #         get_package_share_directory("mechaship_example"),
    #         "param",
    #         "mechaship_navigation.yaml",
    #     ),
    # )
    # mechaship_navigation_arg = DeclareLaunchArgument(
    #     "mechaship_navigation_params",
    #     default_value=mechaship_navigation_params,
    # )
    # mechaship_navigation = Node(
    #     package="mechaship_example",
    #     executable="mechaship_navigation_node",
    #     name="mechaship_navigation_node",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[mechaship_navigation_params],
    # )

    return LaunchDescription(
        [
            # mechaship_detect_arg,
            # mechaship_detect,
            mechaship_classify_arg,
            mechaship_classify,
            # mechaship_navigation_arg,
            # mechaship_navigation,
        ]
    )
    
    