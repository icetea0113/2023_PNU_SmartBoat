import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
     # classify sub 노드
    mechaship_classify_sub_params = LaunchConfiguration(
        "mechaship_classify_sub_params",
        default=os.path.join(
            get_package_share_directory("pnu2023"),
            "param",
            "mechaship_classify_sub.yaml",
        ),
    )
    mechaship_classify_sub_arg = DeclareLaunchArgument(
        "mechaship_classify_params",
        default_value=mechaship_classify_sub_params,
    )
    mechaship_classify_sub = Node(
        package="pnu2023",
        executable="mechaship_classify_sub_node",
        name="mechaship_classify_sub_node",
        output="screen",
        emulate_tty=True,
        parameters=[mechaship_classify_sub_params],
    )

    return LaunchDescription(
        [
            mechaship_classify_sub_arg,
            mechaship_classify_sub,
        ]
    )