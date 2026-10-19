# anymal_c_config

ROS2 launch and world files for the ANYmal C Gazebo simulation.

## Quick Start

Build and source:

    colcon build --symlink-install
    source install/setup.bash

Launch Gazebo:

    ros2 launch anymal_c_config gazebo.launch.py

Send a joint trajectory:

    ros2 topic pub --once /joint_group_position_controller/joint_trajectory \
      trajectory_msgs/msg/JointTrajectory "{\
        joint_names: [LF_HAA_joint, LF_HFE_joint, LF_KFE_joint, LH_HAA_joint, LH_HFE_joint, LH_KFE_joint, RF_HAA_joint, RF_HFE_joint, RF_KFE_joint, RH_HAA_joint, RH_HFE_joint, RH_KFE_joint],\
        points: [{positions: [0.0, 0.2, -0.4, 0.0, 0.2, -0.4, 0.0, 0.2, -0.4, 0.0, 0.2, -0.4], time_from_start: {sec: 1, nanosec: 0}}]\
      }"

Verify controllers:

    ros2 control list_controllers
