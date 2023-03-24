#!/usr/bin/python3

# elfin5_l_moveit.launch.py:
# Launch file for the elfin5_l Robot MoveIt!2 SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument,Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    rviz_arg = DeclareLaunchArgument("rviz_file", default_value="False", description="Load RVIZ file.")
    # *** planning context *** #
        
    robot_description_config = (
        os.path.join(
            get_package_share_directory("elfin5_l_ros2_gazebo"),
            "urdf",
            "elfin5_l.urdf.xacro",
        )
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', robot_description_config,
        ' use_fake_hardware:=false',
        ' use_real_hardware:=true',])

    robot_description = {'robot_description': robot_description_config}

    # load robot description, srdf
    robot_description_semantic_config = load_file(
        "elfin5_l_ros2_moveit2", "config/elfin5_l.srdf"
    )

    robot_description_semantic = {
        "robot_description_semantic":robot_description_semantic_config
    }

    # load kinematics.yaml
    kinematics_yaml = load_yaml(
        "elfin5_l_ros2_moveit2", "config/kinematics.yaml"
    )

    robot_description_kinematics = {"robot_description_kinematics":kinematics_yaml}

    # planning functionality
    ompl_planning_pipeline_config = {
        "move_group":{
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        "start_state_max_bounds_error":0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        "elfin5_l_ros2_moveit2","config/ompl_planning.yaml"
    )

    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # trajectory execution functionality
    moveit_simple_controllers_yaml = load_yaml(
        "elfin5_l_ros2_moveit2", "config/elfin_controllers.yaml"
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates":  True,
        "publish_transforms_updates": True,
    }

    # start move_group & action server
    run_move_group_node = Node(
        package = "moveit_ros_move_group",
        executable = "move_group",
        output = "screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Rviz
    load_rviz = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("elfin5_l_ros2_moveit2"),"launch")
    rviz_full_config = os.path.join(rviz_base,"elfin5_l_moveit2.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_rviz),
    )

    # static tf
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "elfin_base_link"],

    )

    #publish tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("elfin_robot_bringup"),
        "config",
        "elfin_arm_control.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "elfin_arm_controller",
        "joint_state_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [   
            static_tf,
            robot_state_publisher,
            ros2_control_node,
        ]
    )
