import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="anymal_c_config"
    ).find("anymal_c_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="anymal_c_simple_description"
    ).find("anymal_c_simple_description")

    default_model_path = os.path.join(descr_pkg_share, "urdf/anymal_main.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/test_world_2.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_description_path = DeclareLaunchArgument(
        "description_path",
        default_value=default_model_path,
        description="Absolute path to robot xacro/urdf file",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="anymal_c", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("anymal_c_simple_description"),
                "launch",
                "description.launch.py",
            )
        ),
        launch_arguments={
            "description_path": LaunchConfiguration("description_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("anymal_c_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    joint_states_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_states_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_group_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            declare_robot_name,
            declare_lite,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_heading,
            description_ld,
            gazebo_ld,
            joint_states_spawner,
            joint_group_spawner,
        ]
    )
