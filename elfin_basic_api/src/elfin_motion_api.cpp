/*
Created on Mon Nov 27 14:22:43 2017

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

#include "elfin_basic_api/elfin_motion_api.h"

namespace elfin_basic_api {

ElfinMotionAPI::ElfinMotionAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor):
    group_(group), planning_scene_monitor_(planning_scene_monitor), motion_nh_(node)
{
    geometry_msgs::msg::Vector3 gravity_v3;
    gravity_v3.x=0;
    gravity_v3.y=0;
    gravity_v3.z=-9.81;
    dynamics_solver_.reset(new dynamics_solver::DynamicsSolver(group->getRobotModel(), group->getName(), gravity_v3));

    goal_.trajectory.joint_names=group_->getJointNames();

    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nanosec=0;

    joint_goal_sub_=motion_nh_->create_subscription<sensor_msgs::msg::JointState>("joint_goal", rclcpp::SensorDataQoS(),  std::bind(&ElfinMotionAPI::jointGoalCB, this, std::placeholders::_1));
    cart_goal_sub_=motion_nh_->create_subscription<geometry_msgs::msg::PoseStamped>("cart_goal", rclcpp::SensorDataQoS(), std::bind(&ElfinMotionAPI::cartGoalCB, this, std::placeholders::_1));
    cart_path_goal_sub_=motion_nh_->create_subscription<geometry_msgs::msg::PoseArray>("cart_path_goal", rclcpp::SensorDataQoS(), std::bind(&ElfinMotionAPI::cartPathGoalCB, this, std::placeholders::_1));

    get_reference_link_server_=motion_nh_->create_service<std_srvs::srv::SetBool>("get_reference_link", std::bind(&ElfinMotionAPI::getRefLink_cb,this,std::placeholders::_1,std::placeholders::_2));
    get_end_link_server_=motion_nh_->create_service<std_srvs::srv::SetBool>("get_end_link", std::bind(&ElfinMotionAPI::getEndLink_cb,this, std::placeholders::_1, std::placeholders::_2));

    bool torques_publisher_flag;
    motion_nh_->get_parameter_or("torques_publish", torques_publisher_flag, false);
    double torques_publisher_period;
    motion_nh_->get_parameter_or("torques_publish_period", torques_publisher_period, 0.02);
    auto timeout = std::chrono::duration<double, std::milli>(torques_publisher_period*1000);

    if(torques_publisher_flag)
    {
        torques_publisher_=motion_nh_->create_publisher<std_msgs::msg::Float64MultiArray>("desired_torques", 1);
        torques_publisher_timer_ = motion_nh_->create_wall_timer(timeout, std::bind(&ElfinMotionAPI::torquesPubTimer_cb,this));
    }

    end_link_=group_->getEndEffectorLink();
    reference_link_=group_->getPlanningFrame();

    default_tip_link_=group_->getEndEffectorLink();
    root_link_=group_->getPlanningFrame();

    tfBuffer =
      std::make_unique<tf2_ros::Buffer>(motion_nh_->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
}

void ElfinMotionAPI::jointGoalCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(group_->setJointValueTarget(*msg))
    {
        group_->asyncMove();
    }
    else
    {
        RCLCPP_WARN(motion_nh_->get_logger(),"the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::cartGoalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::string reference_link=reference_link_;
    if(!msg->header.frame_id.empty())
    {
        reference_link=msg->header.frame_id;
    }

    if(!updateTransforms(reference_link))
        return;

    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    affine_rootToRef = tf2::transformToEigen(transform_rootToRef_);
    affine_refToRoot=affine_rootToRef.inverse();

    Eigen::Isometry3d affine_tipToEnd;
    affine_tipToEnd=tf2::transformToEigen(transform_tipToEnd_);

    geometry_msgs::msg::TransformStamped tf_pose_tmp;
    Eigen::Isometry3d affine_pose_tmp;
    tf2::fromMsg(msg->pose,affine_pose_tmp);

    Eigen::Isometry3d affine_pose_goal=affine_refToRoot * affine_pose_tmp * affine_tipToEnd;

    if(group_->setPoseTarget(affine_pose_goal))
    {
        group_->asyncMove();
    }
    else
    {
        RCLCPP_WARN(motion_nh_->get_logger(),"the robot cannot execute that motion");
    }
}

void ElfinMotionAPI::trajectoryScaling(moveit_msgs::msg::RobotTrajectory &trajectory, double scale)
{
    if(scale<=0 || scale>=1)
    {
        return;
    }
    for(unsigned int i=0; i<trajectory.joint_trajectory.points.size(); i++)
    {
        trajectory.joint_trajectory.points[i].time_from_start.sec *= (1.0/scale);
        for(unsigned int j=0; j<trajectory.joint_trajectory.points[i].velocities.size(); j++)
        {
            trajectory.joint_trajectory.points[i].velocities[j]*=scale;
        }
        for(unsigned int j=0; j<trajectory.joint_trajectory.points[i].accelerations.size(); j++)
        {
            trajectory.joint_trajectory.points[i].accelerations[j]*=(scale*scale);
        }
    }
}

void ElfinMotionAPI::cartPathGoalCB(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    moveit_msgs::msg::RobotTrajectory cart_path;
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;

    std::vector<geometry_msgs::msg::Pose> pose_goal=msg->poses;

    std::string reference_link=reference_link_;
    if(!msg->header.frame_id.empty())
    {
        reference_link=msg->header.frame_id;
    }

    if(!updateTransforms(reference_link))
        return;

    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    affine_rootToRef = tf2::transformToEigen(transform_rootToRef_);
    affine_refToRoot=affine_rootToRef.inverse();

    Eigen::Isometry3d affine_tipToEnd;
    affine_tipToEnd = tf2::transformToEigen(transform_tipToEnd_);
    geometry_msgs::msg::TransformStamped tf_pose_tmp;
    Eigen::Isometry3d affine_pose_tmp;
    Eigen::Isometry3d affine_goal_tmp;

    for(unsigned int i=0; i<pose_goal.size(); i++)
    {
        tf2::fromMsg(pose_goal[i], affine_pose_tmp);
        affine_goal_tmp=affine_refToRoot * affine_pose_tmp * affine_tipToEnd;
        pose_goal[i] = tf2::toMsg(affine_goal_tmp);
    }

    double fraction=group_->computeCartesianPath(pose_goal, 0.01, 1.5, cart_path);

    if(fraction==-1)
    {
        RCLCPP_WARN(motion_nh_->get_logger(),"there is an error while computing the cartesian path");
        return;
    }

    if(fraction==1)
    {
        RCLCPP_INFO(motion_nh_->get_logger(),"the cartesian path can be %.2f%% acheived", fraction * 100.0);
        trajectoryScaling(cart_path, velocity_scaling_);
        cart_plan.trajectory_=cart_path;
        group_->asyncExecute(cart_plan);
    }
    else
    {
        RCLCPP_INFO(motion_nh_->get_logger(),"the cartesian path can only be %.2f%% acheived and it will not be executed", fraction * 100.0);
    }
}

bool ElfinMotionAPI::getRefLink_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    resp->success=true;
    resp->message=reference_link_;
    return true;
}

bool ElfinMotionAPI::getEndLink_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    resp->success=true;
    resp->message=end_link_;
    return true;
}

void ElfinMotionAPI::torquesPubTimer_cb()
{
    std::vector<double> v_p;
    std::vector<double> v_v;
    std::vector<double> v_a;
    std::vector<geometry_msgs::msg::Wrench> v_w(7);
    std::vector<double> v_t(6);
    for(int i=0;i<6; i++)
    {
        v_v.push_back(0);
        v_a.push_back(0);
    }

    moveit::core::RobotStatePtr current_state=group_->getCurrentState();
    const moveit::core::JointModelGroup* jmp=current_state->getJointModelGroup(group_->getName());
 
    current_state->copyJointGroupPositions(jmp, v_p);

    dynamics_solver_->getTorques(v_p, v_v, v_a, v_w, v_t);

    torques_msg_.data=v_t;

    torques_publisher_->publish(torques_msg_);
}

void ElfinMotionAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
}

void ElfinMotionAPI::setRefFrames(std::string ref_link)
{
    reference_link_=ref_link;
}

void ElfinMotionAPI::setEndFrames(std::string end_link)
{
    end_link_=end_link;
}

bool ElfinMotionAPI::updateTransforms(std::string ref_link)
{
  rclcpp::WallRate r(100);
  int counter=0;
  while(rclcpp::ok())
  {
      try{
        tfBuffer->canTransform(ref_link, root_link_, rclcpp::Time(0), rclcpp::Duration(10.0));
        transform_rootToRef_ = tfBuffer->lookupTransform(ref_link, root_link_, rclcpp::Time(0));
        break;
      }
      catch (tf2::TransformException &ex) {
        r.sleep();
        counter++;
        if(counter>200)
        {
          RCLCPP_ERROR(motion_nh_->get_logger(),"%s",ex.what());
          RCLCPP_INFO(motion_nh_->get_logger(),"Motion planning failed");
          return false;
        }
        continue;
      }
  }

  counter=0;
  while(rclcpp::ok())
  {
      try{
        tfBuffer->canTransform(end_link_, default_tip_link_, rclcpp::Time(0), rclcpp::Duration(10.0));
        transform_tipToEnd_ = tfBuffer->lookupTransform(end_link_, default_tip_link_, rclcpp::Time(0));
        break;
      }
      catch (tf2::TransformException &ex) {
        r.sleep();
        counter++;
        if(counter>200)
        {
          RCLCPP_INFO(motion_nh_->get_logger(),"%s",ex.what());
          RCLCPP_INFO(motion_nh_->get_logger(),"Motion planning failed");
          return false;
        }
        continue;
      }
  }

  return true;
}

}

