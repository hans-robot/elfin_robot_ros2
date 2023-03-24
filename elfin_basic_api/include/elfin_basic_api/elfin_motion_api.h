/*
Created on Mon Nov 27 14:24:30 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#ifndef ELFIN_MOTION_API_H
#define ELFIN_MOTION_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <time.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <elfin_basic_api/elfin_basic_api_const.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

namespace elfin_basic_api{

class ElfinMotionAPI
{
public:
    ElfinMotionAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    void jointGoalCB(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cartGoalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void trajectoryScaling(moveit_msgs::msg::RobotTrajectory &trajectory, double scale);
    void cartPathGoalCB(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    bool getRefLink_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getEndLink_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    void torquesPubTimer_cb();

    void setVelocityScaling(double data);
    void setRefFrames(std::string ref_link);
    void setEndFrames(std::string end_link);

    bool updateTransforms(std::string ref_link);

private:
    moveit::planning_interface::MoveGroupInterfacePtr& group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    dynamics_solver::DynamicsSolverPtr dynamics_solver_;
    rclcpp::Node::SharedPtr root_nh_;
    rclcpp::Node::SharedPtr motion_nh_;
    
    control_msgs::action::FollowJointTrajectory_Goal goal_;

    double velocity_scaling_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cart_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cart_path_goal_sub_;

    std::string end_link_;
    std::string reference_link_;
    std::string default_tip_link_;
    std::string root_link_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    
    geometry_msgs::msg::TransformStamped transform_rootToRef_;
    geometry_msgs::msg::TransformStamped transform_tipToEnd_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_reference_link_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_end_link_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_torques_server_;

    rclcpp::TimerBase::SharedPtr torques_publisher_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher_;
    std_msgs::msg::Float64MultiArray torques_msg_;

};

}

#endif
