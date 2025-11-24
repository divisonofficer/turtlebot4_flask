#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    ip_arg = DeclareLaunchArgument(
        "ip_address", default_value="", description="IP address of the DCS103e device"
    )

    auto_connect_arg = DeclareLaunchArgument(
        "auto_connect",
        default_value="true",
        description="Automatically connect to DCS103e on startup (true by default when IP is provided)",
    )

    # DCS103e controller node
    dcs103e_node = Node(
        package="dcs103e_controller",
        executable="dcs103e_node",
        name="dcs103e_controller",
        output="screen",
        parameters=[
            {
                "ip_address": LaunchConfiguration("ip_address"),
                "auto_connect": LaunchConfiguration("auto_connect"),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ],
    )

    return LaunchDescription([ip_arg, auto_connect_arg, dcs103e_node])
