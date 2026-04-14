"""
Minimal doosan-robot2 bringup for Isaac Sim integration.

Differences from upstream dsr_bringup2_rviz.launch.py:
  * No rviz2 node (Isaac Sim handles visualization).
  * No virtual-mode emulator spawner (we run DRCF emulator separately on the host).

Run:
    ros2 launch /ros2_ws/src/m1013_isaac_bringup.launch.py \
        mode:=real host:=127.0.0.1 port:=12345 model:=m1013

Args mirror upstream: name, host, port, mode, model, color, rt_host.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from dsr_bringup2.utils import read_update_rate, show_git_info


def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="dsr01", description="namespace"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="DRCF host"),
        DeclareLaunchArgument("port", default_value="12345", description="DRCF port"),
        DeclareLaunchArgument("mode", default_value="real", description="real|virtual"),
        DeclareLaunchArgument("model", default_value="m1013", description="robot model"),
        DeclareLaunchArgument("color", default_value="white", description="robot color"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50", description="RT host"),
    ]

    name = LaunchConfiguration("name")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    mode = LaunchConfiguration("mode")
    model = LaunchConfiguration("model")
    color = LaunchConfiguration("color")
    rt_host = LaunchConfiguration("rt_host")

    update_rate = str(read_update_rate())
    show_git_info()

    xacro_path = os.path.join(get_package_share_directory("dsr_description2"), "xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("dsr_description2"), "xacro", model]),
            ".urdf.xacro",
            " name:=", name,
            " host:=", host,
            " rt_host:=", rt_host,
            " port:=", port,
            " mode:=", mode,
            " model:=", model,
            " update_rate:=", update_rate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = [
        PathJoinSubstitution(
            [FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]
        )
    ]

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=name,
        parameters=[robot_description] + robot_controllers,
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        output="both",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_path,
                        "/",
                        model,
                        ".urdf.xacro color:=",
                        color,
                        " name:=", name,
                        " host:=", host,
                        " rt_host:=", rt_host,
                        " port:=", port,
                        " mode:=", mode,
                        " model:=", model,
                        " update_rate:=", update_rate,
                    ]
                )
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=name,
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=name,
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    delay_robot_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        args
        + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_after_joint_state_broadcaster,
        ]
    )
