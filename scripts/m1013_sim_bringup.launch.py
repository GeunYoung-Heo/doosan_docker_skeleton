"""
DRCF-free sim bringup for M1013.

Architecture:
    MoveIt move_group
      → FollowJointTrajectory action
      → dsr_moveit_controller (joint_trajectory_controller)
      → command interfaces
      → mock_components/GenericSystem hardware interface
      → state interfaces → joint_state_broadcaster → /joint_states
      → Isaac Sim bridge (OmniGraph ROS2SubscribeJointState)
      → PhysX M1013 articulation in Isaac Sim viewport

Key differences from upstream dsr_moveit_config_m1013/launch/demo.launch.py:
- No rviz2 (Isaac Sim is the visualizer)
- No dsr_controller2 spawner (no DRCF-backed service interface in sim)
- No run_emulator (DRCF emulator is not used at all)
- No namespace (everything at root) — same as upstream demo
- Explicit start sequencing (control_node → JSB → JTC → move_group)

Run:
    ros2 launch /ros2_ws/src/m1013_sim_bringup.launch.py
or with a different model:
    ros2 launch /ros2_ws/src/m1013_sim_bringup.launch.py model:=m0609
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def _build_moveit_config(model):
    """Shared MoveIt config builder used by both move_group and robot_state_publisher."""
    pkg_name = f"dsr_moveit_config_{model}"
    return (
        MoveItConfigsBuilder(model, "robot_description", pkg_name)
        .robot_description(file_path=f"config/{model}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .to_moveit_configs()
    )


def _move_group(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    moveit_config = _build_moveit_config(model)
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
    ]


def _robot_state_publisher(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    moveit_config = _build_moveit_config(model)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description],
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "model", default_value="m1013",
            description="Robot model (looks up dsr_moveit_config_<model>)",
        ),
    ]

    # Two yaml files loaded for controller_manager + controllers:
    #   1. dsr_controller2.yaml — declares dsr_moveit_controller's *type*
    #      (joint_trajectory_controller/JTC) and sets controller_manager
    #      update_rate to 100 Hz (better joint_state interpolation than the
    #      10 Hz that dsr_moveit_config_m1013's ros2_controllers.yaml uses).
    #   2. dsr_moveit_controller_sim.yaml — overrides the JTC's
    #      command_interfaces to `[position]` only. Needed because the
    #      mock_components hardware (via dsr.ros2_control.xacro) only exposes
    #      position command interface, and JTC's defaults would try to claim
    #      velocity too and fail to activate.
    dsr_controller2_yaml = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"),
        "config",
        "dsr_controller2.yaml",
    ])
    sim_jtc_override_yaml = "/ros2_ws/src/dsr_moveit_controller_sim.yaml"

    # ros2_control_node with mock_components backend. Picks up robot_description
    # by subscribing to /robot_description (published by robot_state_publisher
    # via its robot_description parameter). This is the modern ros2_control
    # "topic-based description" pattern used by upstream demo.launch.py.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[dsr_controller2_yaml, sim_jtc_override_yaml],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_action = OpaqueFunction(function=_robot_state_publisher)
    move_group_action = OpaqueFunction(function=_move_group)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dsr_moveit_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
    )

    # Sequencing: control_node starts → JSB → JTC → move_group
    delay_jsb_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
            ],
        )
    )
    delay_jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dsr_moveit_controller_spawner],
        )
    )
    delay_move_group_after_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[move_group_action],
        )
    )

    # Gripper sim node — provides /onrobot_rg2_ft_node/set_gripper service
    # (same interface as the real gripper_node.py) and publishes finger target
    # on /gripper_finger_target for the Isaac Sim bridge.
    gripper_sim = ExecuteProcess(
        cmd=["python3", "/ros2_ws/src/gripper_sim_node.py"],
        output="screen",
    )

    return LaunchDescription(
        args
        + [
            robot_state_pub_action,
            control_node,
            delay_jsb_after_control,
            delay_jtc_after_jsb,
            delay_move_group_after_jtc,
            gripper_sim,
        ]
    )
