"""Go1 MuJoCo locomotion with the existing Jazzy Nav2 navigation stack."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    workshop = get_package_share_directory("workshop_navigation")
    locomotion = get_package_share_directory("go1_mujoco_locomotion")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    world = LaunchConfiguration("world")
    # world:=factory (default) or world:=maze selects matching scene + map.
    map_file = PathJoinSubstitution([workshop, "maps", ["go1_", world, ".yaml"]])
    scene_file = PathJoinSubstitution([locomotion, "config", ["go1_", world, ".mjb"]])
    is_factory = ["'", world, "' == 'factory'"]
    nav_params = os.path.join(workshop, "config", "go1_mujoco_nav2.yaml")
    rviz_config = os.path.join(workshop, "rviz", "go1_mujoco.rviz")
    locomotion_params = os.path.join(locomotion, "config", "locomotion.yaml")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": nav_params,
            "use_sim_time": "false",
            "use_localization": "True",
            "use_composition": "False",
            "autostart": "True",
        }.items(),
    )
    diagnostic_lib = os.path.join(
        os.environ.get("WORKSHOP_ROOT", "/home/ubuntu/ros2_ws"),
        ".deps", "diagnostic", "lib",
    )
    isolated_nav = GroupAction(actions=[
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            diagnostic_lib + ":" + os.environ.get("LD_LIBRARY_PATH", ""),
        ),
        nav,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="factory"),
        DeclareLaunchArgument("mujoco_viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("auto_goal", default_value="true"),
        DeclareLaunchArgument("map", default_value=map_file),
        DeclareLaunchArgument("scene", default_value=scene_file),
        DeclareLaunchArgument("goal_x", default_value=PythonExpression(["'6.0' if ", *is_factory, " else '14.0'"])),
        DeclareLaunchArgument("goal_y", default_value=PythonExpression(["'4.5' if ", *is_factory, " else '0.0'"])),
        DeclareLaunchArgument("goal_yaw", default_value=PythonExpression(["'3.1416' if ", *is_factory, " else '0.0'"])),
        Node(
            package="go1_mujoco_locomotion",
            executable="locomotion_node",
            name="go1_locomotion",
            output="screen",
            parameters=[
                locomotion_params,
                {
                    "cmd_vel_topic": "/cmd_vel_safe",
                    "scene": LaunchConfiguration("scene"),
                    "publish_tf": False,
                    # A Python MuJoCo ray per beam is expensive; 90 beams at
                    # 10 Hz retain useful aisle coverage without starving the
                    # 50 Hz locomotion policy or Nav2 callbacks.
                    "laser_samples": 90,
                    "scan_rate": 10.0,
                    "mujoco_viewer": ParameterValue(
                        LaunchConfiguration("mujoco_viewer"), value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="workshop_navigation",
            executable="runtime.py",
            name="workshop_runtime",
            output="screen",
        ),
        TimerAction(
            period=5.0,
            actions=[
                isolated_nav,
                Node(
                    package="workshop_navigation",
                    executable="lifecycle_guard.py",
                    name="lifecycle_guard",
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_config],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package="workshop_navigation",
                    executable="goal.py",
                    arguments=[LaunchConfiguration("goal_x"), LaunchConfiguration("goal_y"),
                               "--yaw", LaunchConfiguration("goal_yaw"), "--timeout", "300"],
                    condition=IfCondition(LaunchConfiguration("auto_goal")),
                    output="screen",
                )
            ],
        ),
    ])
