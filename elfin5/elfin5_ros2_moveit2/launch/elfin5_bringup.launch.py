#!/usr/bin/python3

# elfin5_bringup.launch.py
# Single-shot bring-up of the REAL Elfin5 robot on ROS 2 Jazzy. It starts, in
# one launch, everything needed to plan and move the robot:
#   - EtherCAT hardware + controller_manager        (elfin5_moveit.launch.py)
#   - the arm / joint_state controller spawners
#   - then, ONCE THE ROBOT IS READY (controllers active, /joint_states flowing):
#       - MoveIt move_group + RViz                  (elfin5_moveit_rviz.launch.py)
#       - the Elfin basic API node                  (elfin5_basic_api.launch.py)
#       - the "Elfin Control Panel" GUI             (elfin_gui.launch.py)
#
# Sequencing the dependent nodes after the hardware finishes initialising
# (joint position recognition takes ~25 s) avoids RViz/GUI/API errors such as
# "Failed to fetch current robot state" or TF lookup failures at start-up.
#
# Launch arguments:
#   use_gui:=true|false   start the Control Panel GUI (default: true)
#   use_api:=true|false   start the basic API node    (default: true)
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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package_share_dir, relative_launch_path, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_dir, "launch", relative_launch_path)
        ),
        launch_arguments=launch_arguments,
        condition=condition,
    )


def generate_launch_description():
    moveit2_dir = get_package_share_directory("elfin5_ros2_moveit2")
    basic_api_dir = get_package_share_directory("elfin_basic_api")

    use_gui = LaunchConfiguration("use_gui")
    use_api = LaunchConfiguration("use_api")

    # 1. Hardware: EtherCAT + controller_manager (this is the long pole; the
    #    joint position recognition takes ~25 s).
    hardware = _include(moveit2_dir, "elfin5_moveit.launch.py")

    # 2. Controller spawners. They wait for /controller_manager and then load and
    #    activate the controllers as soon as the hardware is up.
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["elfin_arm_controller", "--controller-manager", "/controller_manager"],
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 3. The dependent stack, started only after joint_state_broadcaster is active
    #    (the spawner exits cleanly once the controller is up -> robot is ready).
    #    moveit_rviz is included with spawn_controllers:=false because the
    #    spawners above already did that job.
    moveit_rviz = _include(
        moveit2_dir, "elfin5_moveit_rviz.launch.py",
        launch_arguments={"spawn_controllers": "false"}.items(),
    )
    basic_api = _include(
        moveit2_dir, "elfin5_basic_api.launch.py", condition=IfCondition(use_api),
    )
    gui = _include(
        basic_api_dir, "elfin_gui.launch.py", condition=IfCondition(use_gui),
    )

    start_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[moveit_rviz, basic_api, gui],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="Start the Elfin Control Panel GUI."),
        DeclareLaunchArgument("use_api", default_value="true",
                              description="Start the Elfin basic API node."),
        hardware,
        arm_spawner,
        jsb_spawner,
        start_when_ready,
    ])
