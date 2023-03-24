/*
Created on Mon Sep 17 10:22:24 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
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

#ifndef ELFIN_ETHERCAT_DRIVER_H
#define ELFIN_ETHERCAT_DRIVER_H

#include <elfin_ethercat_driver/elfin_ethercat_client.h>
#include <elfin_ethercat_driver/elfin_ethercat_io_client.h>

namespace elfin_ethercat_driver {

class ElfinEtherCATDriver
{
public:
    ElfinEtherCATDriver(EtherCatManager *manager, std::string driver_name,const rclcpp::Node::SharedPtr& node);
    ~ElfinEtherCATDriver();

    std::shared_ptr<rclcpp::Node> root_nh_, ed_nh_;

    bool getEnableState();
    bool getFaultState();
    bool getMotionState();
    bool getPosAlignState();
    void updateStatus();

    size_t getEtherCATClientNumber();
    ElfinEtherCATClient* getEtherCATClientPtr(size_t n);
    std::string getJointName(size_t n);
    double getReductionRatio(size_t n);
    double getAxisPositionFactor(size_t n);
    double getAxisTorqueFactor(size_t n);
    int32_t getCountZero(size_t n);

    bool recognizePosition();

    bool getTxPDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getRxPDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getCurrentPosition_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getMotionState_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getPosAlignState_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool enableRobot_test();
    bool enableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool disableRobot_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool clearFault_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool recognizePosition_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

    static int32_t getIntFromStr(std::string str);
    void error_log(int line, std::string log, std::string log_param);

private:
        std::vector<ElfinEtherCATClient*> ethercat_clients_;
        std::vector<int64_t> slave_no_;
        std::vector<std::string> joint_names_;
        std::vector<double> reduction_ratios_;
        std::vector<double> axis_position_factors_;
        std::vector<double> axis_torque_factors_;
        std::vector<int64_t> count_zeros_;

        std::vector<ElfinEtherCATIOClient*> ethercat_io_clients_;
        std::vector<int64_t> io_slave_no_;

        std::string driver_name_;
        
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_txpdo_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_rxpdo_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_current_position_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_motion_state_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_pos_align_state_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disable_robot_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_fault_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recognize_position_;

        std_msgs::msg::Bool enable_state_msg_;
        std_msgs::msg::Bool fault_state_msg_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_state_pub_;
        rclcpp::TimerBase::SharedPtr status_timer_;

        std::vector<double> count_rad_factors_;
        double motion_threshold_;
        double pos_align_threshold_;
};

}

#endif
