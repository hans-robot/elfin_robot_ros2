/*
Created on Mon Nov 13 15:20:10 2017

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

#include "elfin_basic_api/elfin_teleop_api.h"

namespace elfin_basic_api {
ElfinTeleopAPI::ElfinTeleopAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor):
    group_(group),planning_scene_monitor_(planning_scene_monitor), teleop_nh_(node)
{

    action_client_=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(teleop_nh_, action_name);
    goal_.trajectory.joint_names=group_->getJointNames();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nanosec=0;
    sub_teleop_joint_command_no_limit_= teleop_nh_->create_subscription<std_msgs::msg::Int64>("elfin_teleop_joint_cmd_no_limit", rclcpp::SensorDataQoS(),  std::bind(&ElfinTeleopAPI::teleopJointCmdNoLimitCB, this, std::placeholders::_1));
    
    joint_teleop_server_=teleop_nh_->create_service<elfin_robot_msgs::srv::SetInt16>("joint_teleop",std::bind(&ElfinTeleopAPI::jointTeleop_cb,this,std::placeholders::_1,std::placeholders::_2));
    cart_teleop_server_=teleop_nh_->create_service<elfin_robot_msgs::srv::SetInt16>("cart_teleop", std::bind(&ElfinTeleopAPI::cartTeleop_cb,this,std::placeholders::_1,std::placeholders::_2));
    home_teleop_server_=teleop_nh_->create_service<std_srvs::srv::SetBool>("home_teleop", std::bind(&ElfinTeleopAPI::homeTeleop_cb,this,std::placeholders::_1,std::placeholders::_2));
    teleop_stop_server_=teleop_nh_->create_service<std_srvs::srv::SetBool>("stop_teleop", std::bind(&ElfinTeleopAPI::teleopStop_cb,this,std::placeholders::_1,std::placeholders::_2));
    // parameter for teleop no limit
    joint_step_=M_PI/100;
    joint_duration_ns_=1e+8;

    // parameter for normal teleop
    joint_speed_limit_=JOINT_SPEED_LIMIT_CONST;
    joint_speed_default_=JOINT_SPEED_DEFAULT_CONST;
    cart_duration_default_=CART_DURATION_DEFAULT_CONST;

    joint_speed_=joint_speed_default_*0.5;
    cart_duration_=cart_duration_default_/0.5;
    group_->setMaxVelocityScalingFactor(0.5);

    resolution_angle_=0.02;
    resolution_linear_=0.005;
    tfBuffer = std::make_unique<tf2_ros::Buffer>(teleop_nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    end_link_=group_->getEndEffectorLink();
    reference_link_=group_->getPlanningFrame();

    default_tip_link_=group_->getEndEffectorLink();
    root_link_=group_->getPlanningFrame();
}

void ElfinTeleopAPI::setVelocityScaling(double data)
{
    velocity_scaling_=data;
    joint_speed_=joint_speed_default_*velocity_scaling_;
    cart_duration_=cart_duration_default_/velocity_scaling_;
    group_->setMaxVelocityScalingFactor(velocity_scaling_);
}

void ElfinTeleopAPI::setRefFrames(std::string ref_link)
{
    reference_link_=ref_link;
}

void ElfinTeleopAPI::setEndFrames(std::string end_link)
{
    end_link_=end_link;
}

void ElfinTeleopAPI::teleopJointCmdNoLimitCB(const std_msgs::msg::Int64::SharedPtr msg)
{
    if(msg->data==0 || abs(msg->data)>goal_.trajectory.joint_names.size())
        return;
    int joint_num=abs(msg->data);
    int symbol;
    if(joint_num==msg->data)
        symbol=1;
    else
        symbol=-1;
    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;
    std::vector<double> position_tmp=group_->getCurrentJointValues();
    position_tmp[joint_num-1]+=symbol*joint_step_;
    point_tmp.positions=position_tmp;
    point_tmp.time_from_start.nanosec=joint_duration_ns_;
    goal_.trajectory.points.push_back(point_tmp);
    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();
}

bool ElfinTeleopAPI::jointTeleop_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetInt16::Request> req,
   const std::shared_ptr<elfin_robot_msgs::srv::SetInt16::Response> resp)
{
    if(req->data==0 || abs(req->data)>goal_.trajectory.joint_names.size())
    {
        resp->success=false;
        resp->message="wrong joint teleop data";
        return true;
    }
    int joint_num=abs(req->data);
    std::vector<double> position_current=group_->getCurrentJointValues();
    std::vector<double> position_goal=position_current;
    double joint_current_position=position_current[joint_num-1];
    std::string direction=goal_.trajectory.joint_names[joint_num-1];
    double sign;
    if(joint_num==req->data)
    {
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->upper;
        direction.append("+");
        sign=1;
    }
    else
    {
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->lower;
        direction.append("-");
        sign=-1;
    }

    double duration_from_speed=fabs(position_goal[joint_num-1]-joint_current_position)/joint_speed_;
    if(duration_from_speed<=0.1)
    {
        resp->success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp->message=result;
        return true;
    }
    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;

    moveit::core::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    moveit::core::RobotState kinematic_state=*kinematic_state_ptr;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();

    std::vector<double> position_tmp=position_current;
    bool collision_flag=false;

    int loop_num=1;
    while(fabs(position_goal[joint_num-1]-position_tmp[joint_num-1])/joint_speed_>0.1)
    {
        position_tmp[joint_num-1]+=joint_speed_*0.1*sign;
        point_tmp.time_from_start=rclcpp::Duration::from_seconds(0.1*loop_num);
        point_tmp.positions=position_tmp;
        goal_.trajectory.points.push_back(point_tmp);
        loop_num++;
    }
    if(!collision_flag)
    {
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
        point_tmp.positions=position_goal;
        point_tmp.time_from_start=rclcpp::Duration::from_seconds(duration_from_speed);;
        goal_.trajectory.points.push_back(point_tmp);
    }
    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();
    resp->success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp->message=result;
    return true;
}

bool ElfinTeleopAPI::cartTeleop_cb(const std::shared_ptr<elfin_robot_msgs::srv::SetInt16::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::SetInt16::Response> resp)
{
    if(req->data==0 || abs(req->data)>6)
    {
        resp->success=false;
        resp->message="wrong cart. teleop data";
        return true;
    }
    int operation_num=abs(req->data);
    int symbol;
    if(operation_num==req->data)
        symbol=1;
    else
        symbol=-1;

    geometry_msgs::msg::PoseStamped current_pose=group_->getCurrentPose(end_link_);
    rclcpp::Rate r(100);
    int counter=0;
    while(rclcpp::ok())
    {
        try{
            tfBuffer -> canTransform(reference_link_, root_link_, rclcpp::Time(0), rclcpp::Duration(10.0)); 
            transform_rootToRef_ = tfBuffer->lookupTransform(reference_link_, root_link_, tf2::TimePointZero);
            break;
        }
        catch (tf2::TransformException &ex) {
          r.sleep();
          counter++;
          if(counter>200)
          {
            RCLCPP_ERROR(teleop_nh_->get_logger(),"%s",ex.what());
            resp->success=false;
            resp->message="can't get pose of reference frame";
            return true;
          }
          continue;
        }
    }
    counter=0;
    while(rclcpp::ok())
    {
        try{
        tfBuffer -> canTransform(end_link_, default_tip_link_, rclcpp::Time(0), rclcpp::Duration(10.0)); 

        transform_tipToEnd_ = tfBuffer ->lookupTransform(end_link_, default_tip_link_, rclcpp::Time(0));

          break;
        }
        catch (tf2::TransformException &ex) {
          r.sleep();
          counter++;
          if(counter>200)
          {
            RCLCPP_ERROR(teleop_nh_->get_logger(),"%s",ex.what());
            resp->success=false;
            resp->message="can't get pose of teleop frame";
            return true;
          }
          continue;
        }
    }
    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    affine_rootToRef = tf2::transformToEigen(transform_rootToRef_);
    affine_refToRoot=affine_rootToRef.inverse();

    Eigen::Isometry3d affine_tipToEnd;
    affine_tipToEnd = tf2::transformToEigen(transform_tipToEnd_);

    geometry_msgs::msg::TransformStamped tf_pose_tmp;
    Eigen::Isometry3d affine_pose_tmp;
    tf2::fromMsg(current_pose.pose,affine_pose_tmp);

    Eigen::Isometry3d affine_current_pose=affine_rootToRef * affine_pose_tmp;
    current_pose.pose = tf2::toMsg(affine_current_pose);

    std::vector<double> current_joint_states=group_->getCurrentJointValues();

    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 y_axis(0, 1, 0);
    tf2::Vector3 z_axis(0, 0, 1);
    double resolution_alpha=resolution_angle_;
    double resolution_delta=resolution_linear_;

    moveit::core::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    moveit::core::RobotState kinematic_state=*kinematic_state_ptr;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();

    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;

    std::string direction;

    bool ik_have_result=true;

    for(int i=0; i<100; i++)
    {
        switch (operation_num) {
        case 1:
            current_pose.pose.position.x+=symbol*resolution_delta;
            if(symbol==1)
                direction="X+";
            else
                direction="X-";
            break;
        case 2:
            current_pose.pose.position.y+=symbol*resolution_delta;
            if(symbol==1)
                direction="Y+";
            else
                direction="Y-";
            break;
        case 3:
            current_pose.pose.position.z+=symbol*resolution_delta;
            if(symbol==1)
                direction="Z+";
            else
                direction="Z-";
            break;
        case 4:
            PoseStampedRotation(current_pose, x_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rx+";
            else
                direction="Rx-";
            break;
        case 5:
            PoseStampedRotation(current_pose, y_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Ry+";
            else
                direction="Ry-";
            break;
        case 6:
            PoseStampedRotation(current_pose, z_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rz+";
            else
                direction="Rz-";
            break;
        default:
            break;
        }

        tf2::fromMsg(current_pose.pose, affine_pose_tmp);

        affine_current_pose=affine_refToRoot * affine_pose_tmp * affine_tipToEnd;
        ik_have_result=kinematic_state.setFromIK(joint_model_group, affine_current_pose, default_tip_link_);
        if(ik_have_result)
        {
            if(goal_.trajectory.points.size()!=i)
                break;
            point_tmp.positions.resize(goal_.trajectory.joint_names.size());
            double biggest_shift=0;
            for(unsigned int j=0; j<goal_.trajectory.joint_names.size(); j++)
            {
                point_tmp.positions[j]=*kinematic_state.getJointPositions(goal_.trajectory.joint_names[j]);
                if(i==0)
                {
                    double shift_tmp=fabs(current_joint_states[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
                else {
                    double shift_tmp=fabs(goal_.trajectory.points[i-1].positions[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
            }
            try{
            if(biggest_shift>cart_duration_*joint_speed_limit_ )
                break;
            }catch(...)
            {
                break;
            }
            point_tmp.time_from_start=rclcpp::Duration::from_seconds((i+1)*cart_duration_);
            goal_.trajectory.points.push_back(point_tmp);
        }
        else
        {
            break;
        }
    }
    if(goal_.trajectory.points.size()==0)
    {
        resp->success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp->message=result;
        return true;
    }
    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();
    resp->success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp->message=result;
    return true;

}

bool ElfinTeleopAPI::homeTeleop_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    moveit::core::RobotStatePtr current_state=group_->getCurrentState();
    std::vector<double> position_current=group_->getCurrentJointValues();

    std::vector<double> position_goal;
    double biggest_shift=0;
    int biggest_shift_num;
    std::vector<double> joint_ratios;
    joint_ratios.resize(position_current.size());
    if(position_current.size()!=goal_.trajectory.joint_names.size())
    {
        resp->success=false;
        resp->message="the joints number is wrong";
        return true;
    }
    for(unsigned int i=0; i<position_current.size(); i++)
    {
        if(fabs(position_current[i])>biggest_shift)
        {
            biggest_shift=fabs(position_current[i]);
            biggest_shift_num=i;
        }
        position_goal.push_back(0);
    }
    if(biggest_shift<0.001)
    {
        resp->success=false;
        std::string result="Elfin is already in home position";
        resp->message=result;
        return true;
    }
    for(unsigned int i=0; i<joint_ratios.size(); i++)
    {
        joint_ratios[i]=position_current[i]/position_current[biggest_shift_num];
    }
    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;
    moveit::core::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    moveit::core::RobotState kinematic_state=*kinematic_state_ptr;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    planning_scene_monitor_->updateFrameTransforms();

    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();

    std::vector<double> position_tmp=position_current;

    bool collision_flag=false;

    int loop_num=1;
    double sign=biggest_shift/position_tmp[biggest_shift_num];
    double duration_from_speed=biggest_shift/joint_speed_;

    while(fabs(position_goal[biggest_shift_num]-position_tmp[biggest_shift_num])/joint_speed_>0.1)
    {
        for(unsigned int i=0; i<position_tmp.size(); i++)
        {
            position_tmp[i]-=joint_speed_*0.1*sign*joint_ratios[i];
        }

        kinematic_state.setJointGroupPositions(joint_model_group, position_tmp);
        if(plan_scene->isStateColliding(kinematic_state, group_->getName()))
        {
            if(loop_num==1)
            {
                resp->success=false;
                std::string result="Stop going to home position";
                resp->message=result;
                return true;
            }
            collision_flag=true;
            break;
        }
        point_tmp.time_from_start = rclcpp::Duration::from_seconds(0.1*loop_num);
        point_tmp.positions=position_tmp;
        goal_.trajectory.points.push_back(point_tmp);
        loop_num++;
    }

    if(!collision_flag)
    {
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal);

        if(!plan_scene->isStateColliding(kinematic_state, group_->getName()))
        {
            point_tmp.positions=position_goal;
            // rclcpp::Duration::from_seconds dur(duration_from_speed);

            point_tmp.time_from_start=rclcpp::Duration::from_seconds(duration_from_speed);;
            goal_.trajectory.points.push_back(point_tmp);
        }
        else if(loop_num==1)
        {
            resp->success=false;
            std::string result="Stop going to home position";
            resp->message=result;
            return true;
        }
    }

    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();

    resp->success=true;
    std::string result="robot is moving to home position";
    resp->message=result;
    return true;

}

bool ElfinTeleopAPI::teleopStop_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    goal_.trajectory.points.clear();
    action_client_->async_send_goal(goal_);

    resp->success=true;
    resp->message="stop moving";
    return true;
}

void ElfinTeleopAPI::PoseStampedRotation(geometry_msgs::msg::PoseStamped &pose_stamped, const tf2::Vector3 &axis, double angle)
{
    tf2::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                 pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    tf2::Quaternion q_2(axis, angle);
    tf2::Matrix3x3 m(q_1);
    tf2::Matrix3x3 m_2(q_2);
    m_2.operator *=(m);
    double r, p, y;
    m_2.getRPY(r,p,y);

    q_2.setRPY(r, p, y);
    pose_stamped.pose.orientation.x=q_2.getX();
    pose_stamped.pose.orientation.y=q_2.getY();
    pose_stamped.pose.orientation.z=q_2.getZ();
    pose_stamped.pose.orientation.w=q_2.getW();
}

} // end namespace elfin_basic_api