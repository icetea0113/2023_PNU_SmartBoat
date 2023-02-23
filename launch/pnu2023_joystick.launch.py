#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node, PushRosNamespace, LifecycleNode


def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    teleop_joystick_node = Node(
        package="pnu2023",
        executable="mechaship_teleop_joystick",
        name="mechaship_teleop_joystick",
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_joystick_node,
        ]
    )
