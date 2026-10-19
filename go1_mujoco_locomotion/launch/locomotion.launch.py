#!/usr/bin/env python3
"""单独启动 Go1 运动层节点。

    ros2 launch go1_mujoco_locomotion locomotion.launch.py
    ros2 launch go1_mujoco_locomotion locomotion.launch.py use_sim_time:=true
    ros2 launch go1_mujoco_locomotion locomotion.launch.py publish_tf:=false
    ros2 launch go1_mujoco_locomotion locomotion.launch.py rviz:=true

★ 这个 launch **只起运动层**，不起 Nav2 —— 就是为了方便把它塞进
  别人的导航栈：你那边照常起 Nav2，这边起运动层，靠 cmd_vel / odom / scan 对接。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("go1_mujoco_locomotion")
    params = os.path.join(share, "config", "locomotion.yaml")

    def arg(name, default, desc=""):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    def cfg(name):
        return LaunchConfiguration(name)

    loco = Node(
        package="go1_mujoco_locomotion",
        executable="locomotion_node",
        name="go1_locomotion",
        output="screen",
        parameters=[
            params,
            {
                # ★ 时间基准必须全栈统一：本节点用墙钟，
                #   若你那边是 Gazebo 且有 /clock，把这里设成 true。
                "use_sim_time": ParameterValue(cfg("use_sim_time"), value_type=bool),
                "publish_tf": ParameterValue(cfg("publish_tf"), value_type=bool),
                "cmd_vel_topic": cfg("cmd_vel_topic"),
                "odom_topic": cfg("odom_topic"),
                "scan_topic": cfg("scan_topic"),
                "laser_samples": ParameterValue(cfg("laser_samples"), value_type=int),
                "scan_rate": ParameterValue(cfg("scan_rate"), value_type=float),
                "mujoco_viewer": ParameterValue(cfg("mujoco_viewer"), value_type=bool),
            },
        ],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz",
        arguments=["-d", os.path.join(share, "config", "locomotion.rviz")],
        condition=IfCondition(cfg("rviz")), output="screen",
    )

    return LaunchDescription([
        arg("use_sim_time", "false", "全栈必须统一；有 /clock 才设 true"),
        arg("publish_tf", "true", "是否由本节点发布 odom->base_link"),
        arg("cmd_vel_topic", "cmd_vel", "订阅的速度指令话题"),
        arg("odom_topic", "odom", "发布的里程计话题"),
        arg("scan_topic", "scan", "发布的激光话题"),
        arg("laser_samples", "360", "激光线数"),
        arg("scan_rate", "10.0", "激光更新频率（Hz）"),
        arg("rviz", "false", "是否附带启动 RViz"),
        arg("mujoco_viewer", "false", "打开 MuJoCo 实时仿真窗口"),
        loco,
        rviz,
    ])
