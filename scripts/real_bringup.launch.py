"""
Real-robot bringup for Doosan M1013 + OnRobot RG2-FT V2.

Includes the official Doosan MoveIt launch (dsr_moveit_config_m1013/start.launch.py)
and adds the gripper driver node so both come up with a single command.

Usage:
  ros2 launch /ros2_ws/src/real_bringup.launch.py \
      host:=192.168.137.100 gripper_host:=192.168.1.1

All Doosan arguments (host, port, mode, model, …) are forwarded as-is.
The gripper_host argument is consumed here and not forwarded to Doosan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Arguments forwarded to Doosan start.launch.py
    # ------------------------------------------------------------------
    arm_args = [
        DeclareLaunchArgument("name",    default_value="",               description="Robot namespace"),
        DeclareLaunchArgument("host",    default_value="192.168.137.100", description="Robot controller IP"),
        DeclareLaunchArgument("port",    default_value="12345",           description="Robot controller port"),
        DeclareLaunchArgument("mode",    default_value="real",            description="Operation mode (real/virtual)"),
        DeclareLaunchArgument("model",   default_value="m1013",           description="Robot model"),
        DeclareLaunchArgument("color",   default_value="white",           description="Robot color"),
        DeclareLaunchArgument("gui",     default_value="false",           description="Start RViz2"),
        DeclareLaunchArgument("gz",      default_value="false",           description="Use Gazebo sim"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50",  description="Robot RT IP"),
    ]

    # ------------------------------------------------------------------
    # Gripper-specific argument (NOT forwarded to Doosan)
    # ------------------------------------------------------------------
    gripper_host_arg = DeclareLaunchArgument(
        "gripper_host",
        default_value="192.168.1.1",
        description="OnRobot Compute Box IP",
    )

    # ------------------------------------------------------------------
    # Doosan official MoveIt launch
    # ------------------------------------------------------------------
    doosan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dsr_moveit_config_m1013"),
                "launch",
                "start.launch.py",
            )
        ),
        launch_arguments={
            "name":    LaunchConfiguration("name"),
            "host":    LaunchConfiguration("host"),
            "port":    LaunchConfiguration("port"),
            "mode":    LaunchConfiguration("mode"),
            "model":   LaunchConfiguration("model"),
            "color":   LaunchConfiguration("color"),
            "gui":     LaunchConfiguration("gui"),
            "gz":      LaunchConfiguration("gz"),
            "rt_host": LaunchConfiguration("rt_host"),
        }.items(),
    )

    # ------------------------------------------------------------------
    # OnRobot RG2-FT V2 gripper node
    # ------------------------------------------------------------------
    gripper_node = Node(
        package="onrobot_rg2_ft",
        executable="gripper_node",
        name="onrobot_rg2_ft_node",
        parameters=[{
            "host": LaunchConfiguration("gripper_host"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        arm_args
        + [gripper_host_arg, doosan_launch, gripper_node]
    )
