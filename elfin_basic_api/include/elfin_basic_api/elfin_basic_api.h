/*
Created on Mon Dec 15 10:38:07 2017

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

#ifndef ELFIN_BASIC_API_H
#define ELFIN_BASIC_API_H

#include <elfin_basic_api/elfin_teleop_api.h>
#include <elfin_basic_api/elfin_motion_api.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <elfin_robot_msgs/srv/set_string.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/msg/hardware_interface.hpp>
#include <string.h>
namespace elfin_basic_api {

class ElfinBasicAPI
{
public:
    ElfinBasicAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    ~ElfinBasicAPI();
    
    rcl_interfaces::msg::SetParametersResult dynamicReconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

    void setVelocityScaling(double data);

    bool setVelocityScaling_cb(elfin_robot_msgs::srv::SetFloat64::Request req, elfin_robot_msgs::srv::SetFloat64::Response resp);
    bool updateVelocityScaling_cb(std_srvs::srv::SetBool::Request req, std_srvs::srv::SetBool::Response resp);

    bool setRefLink_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp);
    bool setEndLink_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp);
    bool enableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool disableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

    bool stopActCtrlrs(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool startElfinCtrlr(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

    void set_vel_cb(const std_msgs::msg::Float32::SharedPtr msg);

private:
    moveit::planning_interface::MoveGroupInterfacePtr& group_;
    planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor_;

    rclcpp::Node::SharedPtr root_nh_;
    rclcpp::Node::SharedPtr local_nh_;  

    ElfinTeleopAPI *teleop_api_;
    ElfinMotionAPI *motion_api_;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;

    control_msgs::action::FollowJointTrajectory::Goal goal_;

    double velocity_scaling_;

    rclcpp::Service<elfin_robot_msgs::srv::SetString>::SharedPtr set_ref_link_server_;
    rclcpp::Service<elfin_robot_msgs::srv::SetString>::SharedPtr set_end_link_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disable_robot_server_;

    std::string elfin_controller_init_name = "elfin_controller_name";
    std::string elfin_controller_name_;
    std::vector<std::string> controller_joint_names_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr get_motion_state_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr get_pos_align_state_client_;

    //std_srvs::srv::SetBool::Request::SharedPtr raw_enable_robot_request_;
    //std_srvs::srv::SetBool::Response::SharedPtr raw_enable_robot_response_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr raw_enable_robot_client_;


    std_srvs::srv::SetBool::Request::SharedPtr raw_disable_robot_request_;
    std_srvs::srv::SetBool::Response::SharedPtr raw_disable_robot_response_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr raw_disable_robot_client_;


    std_msgs::msg::String ref_link_name_msg_;
    std_msgs::msg::String end_link_name_msg_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  get_vel;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ref_link_name_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr end_link_name_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

};

}

#endif
