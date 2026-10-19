import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    robot_name        = LaunchConfiguration("robot_name")
    use_sim_time      = LaunchConfiguration("use_sim_time")
    gui               = LaunchConfiguration("gui")
    world_init_x      = LaunchConfiguration("world_init_x")
    world_init_y      = LaunchConfiguration("world_init_y")
    world_init_z      = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world      = LaunchConfiguration("world")

    gz_pkg_share = get_package_share_directory("anymal_c_gazebo")

    # ── Declare arguments ────────────────────────────────────────────────
    declare_robot_name        = DeclareLaunchArgument("robot_name",        default_value="anymal_c")
    declare_use_sim_time      = DeclareLaunchArgument("use_sim_time",      default_value="True")
    declare_gui               = DeclareLaunchArgument("gui",               default_value="True")
    declare_gazebo_world      = DeclareLaunchArgument(
        "world",
        # Default to the Harmonic .sdf world; callers can override with test_world_2.sdf
        default_value=os.path.join(gz_pkg_share, "worlds/default.sdf"),
    )
    declare_world_init_x      = DeclareLaunchArgument("world_init_x",      default_value="0.0")
    declare_world_init_y      = DeclareLaunchArgument("world_init_y",      default_value="0.0")
    declare_world_init_z      = DeclareLaunchArgument("world_init_z",      default_value="0.6")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    # ── Make worlds/ findable by gz-sim resource loader ─────────────────
    # GZ_SIM_RESOURCE_PATH is the Harmonic equivalent of GAZEBO_MODEL_PATH.
    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(gz_pkg_share, "worlds"),
    )

    # ── Launch gz-sim (Harmonic) ─────────────────────────────────────────
    # ros_gz_sim ships gz_sim.launch.py which wraps `gz sim`.
    # -r  → start running immediately (replaces Classic --pause=false).
    # gui is controlled by the `gui` argument passed to gz_sim.launch.py.
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            # Pass the world file path; gz-sim accepts absolute paths directly.
            "gz_args": ["-r ", gazebo_world],
            # Shut down the whole launch when gz-sim exits.
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Spawn robot from robot_description topic ─────────────────────────
    # ros_gz_sim/create replaces gazebo_ros/spawn_entity.py.
    # The URDF is published on /robot_description by robot_state_publisher
    # (started in bringup.launch.py), so we reference it by topic.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  robot_name,
            "-topic", "/robot_description",
            "-x",     world_init_x,
            "-y",     world_init_y,
            "-z",     world_init_z,
            "-R",     "0",
            "-P",     "0",
            "-Y",     world_init_heading,
        ],
    )

    # ── Bridge Gazebo Sim and ROS 2 topics ────────────────────────────────
    # We bridge:
    # 1. /clock (simulation time) -> critical for controllers to get updates
    # 2. /gz/imu -> /imu/data (IMU sensor)
    # 3. /model/anymal_c/odometry -> /odom/ground_truth (Odometry)
    # 4. /gz/contact_lf -> /contact_force_sensors/lf (Contacts)
    # 5. /gz/contact_rf -> /contact_force_sensors/rf (Contacts)
    # 6. /gz/contact_lh -> /contact_force_sensors/lh (Contacts)
    # 7. /gz/contact_rh -> /contact_force_sensors/rh (Contacts)
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/gz/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/anymal_c/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/anymal/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/gz/contact_lf@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/gz/contact_rf@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/gz/contact_lh@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/gz/contact_rh@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        remappings=[
            ("/gz/imu", "/imu/data"),
            ("/model/anymal_c/odometry", "/odom/ground_truth"),
            ("/model/anymal/odometry", "/odom/ground_truth"),
            ("/gz/contact_lf", "/contact_force_sensors/lf"),
            ("/gz/contact_rf", "/contact_force_sensors/rf"),
            ("/gz/contact_lh", "/contact_force_sensors/lh"),
            ("/gz/contact_rh", "/contact_force_sensors/rh"),
        ],
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_gui,
            declare_gazebo_world,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            set_gz_resource_path,
            gz_sim,
            spawn_robot,
            ros_gz_bridge,
        ]
    )

