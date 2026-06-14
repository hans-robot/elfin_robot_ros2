#!/usr/bin/python3

# elfin5_bringup.launch.py
# Single-shot bring-up of the REAL Elfin5 robot on ROS 2 Jazzy. It starts, in
# one launch, everything needed to plan and move the robot:
#   - EtherCAT hardware + controller_manager + controllers   (elfin5_moveit.launch.py)
#   - MoveIt move_group + RViz + controller spawners         (elfin5_moveit_rviz.launch.py)
#   - the Elfin basic API node                               (elfin5_basic_api.launch.py)
#   - the "Elfin Control Panel" GUI                          (elfin_gui.launch.py)
#
# Run as root: SOEM needs raw socket access to the EtherCAT NIC, and keeping
# every node under the same user avoids cross-user DDS issues. For the RViz/GUI
# windows, allow root to use the X display first (see README):
#   xhost +SI:localuser:root
#
# Set the gripper / I/O and NIC parameters in
# elfin_robot_bringup/config/elfin_arm_control.yaml before launching.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package_share_dir, relative_launch_path):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, "launch", relative_launch_path)
        )
    )


def generate_launch_description():
    moveit2_dir = get_package_share_directory("elfin5_ros2_moveit2")
    basic_api_dir = get_package_share_directory("elfin_basic_api")

    return LaunchDescription([
        # EtherCAT hardware + controller_manager (no move_group / spawners here)
        _include(moveit2_dir, "elfin5_moveit.launch.py"),
        # move_group + RViz + the controller spawners
        _include(moveit2_dir, "elfin5_moveit_rviz.launch.py"),
        # Elfin basic API (teleop / cartesian / servo services)
        _include(moveit2_dir, "elfin5_basic_api.launch.py"),
        # Elfin Control Panel GUI
        _include(basic_api_dir, "elfin_gui.launch.py"),
    ])
