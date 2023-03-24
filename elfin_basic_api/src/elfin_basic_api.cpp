/*
Created on Mon Dec 15 10:58:42 2017

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

#include <elfin_basic_api/elfin_basic_api.h>

namespace elfin_basic_api {

ElfinBasicAPI::ElfinBasicAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor):
    local_nh_(node), group_(group), planning_scene_monitor_(planning_scene_monitor)
{   
    
    teleop_api_=new ElfinTeleopAPI(local_nh_,group, action_name, planning_scene_monitor);
    motion_api_=new ElfinMotionAPI(local_nh_,group, planning_scene_monitor);
    auto  callback_group_service_ = local_nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    local_nh_->set_on_parameters_set_callback(std::bind(&ElfinBasicAPI::dynamicReconfigureCallback, this, std::placeholders::_1));

    set_ref_link_server_=local_nh_->create_service<elfin_robot_msgs::srv::SetString>("set_reference_link", std::bind(&ElfinBasicAPI::setRefLink_cb,this,std::placeholders::_1,std::placeholders::_2));
    set_end_link_server_=local_nh_->create_service<elfin_robot_msgs::srv::SetString>("set_end_link", std::bind(&ElfinBasicAPI::setEndLink_cb,this,std::placeholders::_1,std::placeholders::_2));
    enable_robot_server_=local_nh_->create_service<std_srvs::srv::SetBool>("/elfin_basic_api/enable_robot", std::bind(&ElfinBasicAPI::enableRobot_cb, this, std::placeholders::_1,std::placeholders::_2));
    disable_robot_server_=local_nh_->create_service<std_srvs::srv::SetBool>("/elfin_basic_api/disable_robot", std::bind(&ElfinBasicAPI::disableRobot_cb,this,std::placeholders::_1,std::placeholders::_2));

    local_nh_->get_parameter_or("controller_name",elfin_controller_name_, elfin_controller_init_name);

    switch_controller_client_=local_nh_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    list_controllers_client_=local_nh_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    get_motion_state_client_=local_nh_->create_client<std_srvs::srv::SetBool>("/get_motion_state");
    get_pos_align_state_client_=local_nh_->create_client<std_srvs::srv::SetBool>("/get_pos_align_state");
    raw_enable_robot_client_=local_nh_->create_client<std_srvs::srv::SetBool>("/enable_robot");
    raw_disable_robot_client_=local_nh_->create_client<std_srvs::srv::SetBool>("/disable_robot");

    ref_link_name_publisher_=local_nh_->create_publisher<std_msgs::msg::String>("reference_link_name", 1);
    end_link_name_publisher_=local_nh_->create_publisher<std_msgs::msg::String>("end_link_name", 1);

    ref_link_name_msg_.data=group_->getPlanningFrame();
    end_link_name_msg_.data=group_->getEndEffectorLink();

    ref_link_name_publisher_->publish(ref_link_name_msg_);
    end_link_name_publisher_->publish(end_link_name_msg_);

    get_vel = local_nh_->create_subscription<std_msgs::msg::Float32>("vel", rclcpp::SensorDataQoS(),  std::bind(&ElfinBasicAPI::set_vel_cb, this, std::placeholders::_1));
    double vel;

    local_nh_->get_parameter("velocity_scaling",vel);
    tfBuffer =
      std::make_unique<tf2_ros::Buffer>(local_nh_->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    rclcpp::spin(local_nh_);
}

ElfinBasicAPI::~ElfinBasicAPI()
{
    if(teleop_api_ != NULL)
        delete teleop_api_;
    if(motion_api_ != NULL)
        delete motion_api_;
}

rcl_interfaces::msg::SetParametersResult ElfinBasicAPI::dynamicReconfigureCallback(const std::vector<rclcpp::Parameter> &parameter)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &parameter : parameter)
    {
        if (parameter.get_name() == "velocity_scaling" &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            setVelocityScaling(parameter.as_double());
            RCLCPP_INFO(local_nh_->get_logger(), "Parameter 'velocity_scaling' changed: %f", parameter.as_double());
        }
    }
    return result;
}

void ElfinBasicAPI::set_vel_cb(const std_msgs::msg::Float32::SharedPtr msg)
{
    double new_vel = msg->data;
    setVelocityScaling(new_vel);
}

void ElfinBasicAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
    teleop_api_->setVelocityScaling(velocity_scaling_);
    motion_api_->setVelocityScaling(velocity_scaling_);
}

bool ElfinBasicAPI::setRefLink_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp)
{   
    if(!tfBuffer->_frameExists(req->data))
    {
        resp->success=false;
        std::string result="There is no frame named ";
        result.append(req->data);
        resp->message=result;
        return true;
    }
    teleop_api_->setRefFrames(req->data);
    motion_api_->setRefFrames(req->data);
    ref_link_name_msg_.data=req->data;
    ref_link_name_publisher_->publish(ref_link_name_msg_);

    resp->success=true;
    resp->message="Setting reference link succees";
    return true;
}

bool ElfinBasicAPI::setEndLink_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp)
{
    if(!tfBuffer->_frameExists(req->data))
    {
        resp->success=false;
        std::string result="There is no frame named ";
        result.append(req->data);
        resp->message=result;
        return true;
    }

    teleop_api_->setEndFrames(req->data);
    motion_api_->setEndFrames(req->data);

    end_link_name_msg_.data=req->data;
    end_link_name_publisher_->publish(end_link_name_msg_);

    resp->success=true;
    resp->message="Setting end link succeed";
    return true;
}

bool ElfinBasicAPI::enableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    // Check request
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }

    // Check if there is a real driver
    if(!raw_enable_robot_client_->service_is_ready())
    {
        resp->message="there is no real driver running";
        resp->success=false;
        return true;
    }
    
    auto req_tmp = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto resp_tmp = std::make_shared<std_srvs::srv::SetBool::Response>();

    // Stop active controllers
    if(!stopActCtrlrs(resp_tmp))
    {
        resp->success=resp_tmp->success;
        resp->message=resp_tmp->message;
        return true;
    }

    //usleep(500000);
    // Check motion state
    if(!get_motion_state_client_->service_is_ready())
    {
        resp->message="there is no get_motion_state service";
        resp->success=false;
        return true;
    }
    req_tmp->data=true;
    auto resp_tmp_res = get_motion_state_client_->async_send_request(req_tmp);
    resp_tmp = resp_tmp_res.get();
    if(resp_tmp->success)
    {
        resp->message="failed to enable the robot, it's moving";
        resp->success=false;
        return true;
    }
   
    // Check position alignment state
    if(!get_pos_align_state_client_->service_is_ready())
    {
        resp->message="there is no get_pos_align_state service";
        resp->success=false;
        return true;
    }
    req_tmp->data=true;
    resp_tmp_res = get_pos_align_state_client_->async_send_request(req_tmp);
    resp_tmp = resp_tmp_res.get();
    if(!resp_tmp->success)
    {
        resp->message="failed to enable the robot, commands aren't aligned with actual positions";
        resp->success=false;
        return true;
    }

    // Check enable service
    if(!raw_enable_robot_client_->service_is_ready())
    {
        resp->message="there is no real driver running";
        resp->success=false;
        return true;
    }

    // Enable servos
    auto raw_enable_robot_request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto raw_enable_robot_response_ = std::make_shared<std_srvs::srv::SetBool::Response>();
    raw_enable_robot_request_->data=true;

    auto raw_enable_res = raw_enable_robot_client_->async_send_request(raw_enable_robot_request_);

    raw_enable_robot_response_ = raw_enable_res.get();
    resp->message="robot is enable";
    resp->success=true;
    // Start default controller
    if(!startElfinCtrlr(resp_tmp))
    {
        resp->message=resp_tmp->message;
        resp->success=resp_tmp->success;
       return true;
    }

    return true;
}

bool ElfinBasicAPI::disableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    // Check request
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }

    // Check disable service
    if(!raw_disable_robot_client_->service_is_ready())
    {
        resp->message="there is no real driver running";
        resp->success=false;
        return true;
    }

    // Disable servos
    raw_disable_robot_request_->data=true;
    auto raw_disable_res = raw_disable_robot_client_->async_send_request(raw_disable_robot_request_);
    raw_disable_robot_response_ = raw_disable_res.get();
    resp->message=raw_disable_robot_response_->message;
    resp->success=raw_disable_robot_response_->success;

    auto resp_tmp = std::make_shared<std_srvs::srv::SetBool::Response>();

    // Stop active controllers
    //if(!stopActCtrlrs(resp_tmp))
    //{
    //    resp->success=resp_tmp->success;
    //    resp->message=resp_tmp->message;
    //    return true;
    //}

    return true;
}

bool ElfinBasicAPI::stopActCtrlrs(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    // Check list controllers service
    if(!list_controllers_client_->service_is_ready())
    {
        resp->message="there is no controller manager";
        resp->success=false;
        return false;
    }

    // Find controllers to stop
    auto list_controllers_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto list_control_res = list_controllers_client_->async_send_request(list_controllers_request);
    auto list_controllers_response = list_control_res.get();
    std::vector<std::string> controllers_to_stop;
    controllers_to_stop.clear();

    controller_joint_names_.clear();
    for(unsigned int i=0; i<list_controllers_response->controller.size(); i++)
    {
        std::string name_tmp=list_controllers_response->controller[i].name;
        std::vector<std::string> resrc_tmp = list_controllers_response->controller[i].claimed_interfaces;
        if(strcmp(name_tmp.c_str(), elfin_controller_name_.c_str())==0)
        {
            for(unsigned int j=0; j<resrc_tmp.size(); j++)
            {
                controller_joint_names_.insert(controller_joint_names_.end(), resrc_tmp[j]);
            }
            break;
        }
    }

    for(unsigned int i=0; i<list_controllers_response->controller.size(); i++)
    {
        std::string state_tmp=list_controllers_response->controller[i].state;
        std::string name_tmp=list_controllers_response->controller[i].name;
        std::vector<std::string> resrc_tmp=list_controllers_response->controller[i].claimed_interfaces;
        if(strcmp(state_tmp.c_str(), "running")==0)
        {
            bool break_flag=false;
            for(unsigned int j=0; j<resrc_tmp.size(); j++)
            {
                for(unsigned int k=0; k<controller_joint_names_.size(); k++)
                {
                    if(strcmp(resrc_tmp[j].c_str(),controller_joint_names_[k].c_str())!=0)
                    {
                        break_flag=true;
                        controllers_to_stop.push_back(name_tmp);
                    }
                    if(break_flag)
                    {
                        break;
                    }
                }
                if(break_flag)
                {
                    break;
                }
            }
        }
    }

    // Stop active controllers
    if(controllers_to_stop.size()>0)
    {
        // Check switch controller service
        if(!switch_controller_client_->service_is_ready())
        {
            resp->message="there is no controller manager";
            resp->success=false;
            return false;
        }

        // Stop active controllers
        auto switch_controller_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        switch_controller_request->start_controllers.clear();
        switch_controller_request->stop_controllers=controllers_to_stop;
        switch_controller_request->strictness=switch_controller_request->STRICT;

        auto switch_control_res = switch_controller_client_->async_send_request(switch_controller_request);

        auto switch_controller_response = switch_control_res.get();
        if(!switch_controller_response->ok)
        {
            resp->message="Failed to stop active controllers";
            resp->success=false;
            return false;
        }
    }

    return true;
}

bool ElfinBasicAPI::startElfinCtrlr(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    // Check switch controller service
    if(!switch_controller_client_->service_is_ready())
    {
        resp->message="there is no controller manager";
        resp->success=false;
        return false;
    }

    // Start active controllers
    auto switch_controller_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    switch_controller_request->start_controllers.clear();
    switch_controller_request->start_controllers.push_back(elfin_controller_name_);
    switch_controller_request->stop_controllers.clear();
    switch_controller_request->strictness=switch_controller_request->STRICT;

    auto switch_controller_res = switch_controller_client_->async_send_request(switch_controller_request);
    auto switch_controller_response = switch_controller_res.get();
    if(!switch_controller_response->ok)
    {
        resp->message="Failed to start the default controller";
        resp->success=false;
        return false;
    }

    return true;
}

}
