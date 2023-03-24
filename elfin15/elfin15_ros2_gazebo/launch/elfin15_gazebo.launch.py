#!/usr/bin/python3


# elfin15_gazebo.launch.py:
# Launch file for the elfin15 Robot GAZEBO SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    elfin15_ros2_gazebo = os.path.join(
        get_package_share_directory('elfin15_ros2_gazebo'),
        'worlds',
        'elfin15.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': elfin15_ros2_gazebo}.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # elfin15 Description file package:
    elfin15_description_path = os.path.join(
        get_package_share_directory('elfin15_ros2_gazebo'))
    # elfin15 ROBOT urdf file path:
    xacro_file = os.path.join(elfin15_description_path,
                              'urdf',
                              'elfin15.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_fake_hardware:=false',
        ' use_real_hardware:=false',])

    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'elfin15',"-x", "0.0", "-y", "0.0", "-z", "0.1"],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint STATE Controller:
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
        output='screen'
    )
    # Joint TRAJECTORY Controller:
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
        output='screen'
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
        spawn_entity
    ])