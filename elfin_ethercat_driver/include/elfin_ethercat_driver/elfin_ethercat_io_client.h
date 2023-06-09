/*
Created on Tus Nov 17 15:36 2020

@author: Burb

 Software License Agreement (BSD License)

 Copyright (c) 2020, Han's Robot Co., Ltd.
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
// author: Burb

#ifndef ELFIN_ETHERCAT_IO_CLIENT_H
#define ELFIN_ETHERCAT_IO_CLIENT_H

#include<rclcpp/rclcpp.hpp>
#include <vector>
#include <elfin_ethercat_driver/elfin_ethercat_manager.h>
#include "elfin_robot_msgs/srv/elfin_iod_read.hpp"
#include "elfin_robot_msgs/srv/elfin_iod_write.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace elfin_io_txpdo {

const int DIGITAL_INPUT=0;
const int ANALOG_INPUT_CHANNEL1=1;
const int ANALOG_INPUT_CHANNEL2=2;
const int SMART_CAMERA_X=3;
const int SMART_CAMERA_Y=4;

}

namespace elfin_io_rxpdo {

const int DIGITAL_OUTPUT=0;

}

namespace  elfin_ethercat_driver {

class ElfinEtherCATIOClient{

private:
    EtherCatManager* manager_;

    std::shared_ptr<rclcpp::Node> n_, io_nh_;
    std::vector<ElfinPDOunit> pdo_input; // txpdo
    std::vector<ElfinPDOunit> pdo_output; //rxpdo
    int slave_no_;

    rclcpp::Service<elfin_robot_msgs::srv::ElfinIODRead>::SharedPtr read_sdo_;
    rclcpp::Service<elfin_robot_msgs::srv::ElfinIODRead>::SharedPtr read_do_;
    rclcpp::Service<elfin_robot_msgs::srv::ElfinIODWrite>::SharedPtr write_sdo_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_txsdo_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr get_rxsdo_server_;


public:
    ElfinEtherCATIOClient(EtherCatManager* manager, int slave_no, const rclcpp::Node::SharedPtr& nh, std::string io_port_name);
    ~ElfinEtherCATIOClient();
    int32_t readSDO_unit(int n); // 20201117
    int32_t readDO_unit(int n); // 20201130
    void writeOutput_unit(int n, int32_t val);
    int32_t writeSDO_unit(int n); // 20201117
    
    int16_t readInput_unit(int n);
    int32_t readOutput_unit(int n);
    
    std::string getTxSDO();
    std::string getRxSDO();

    bool readSDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Response> resp); // 20201117
    bool readDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Response> resp); // 20201130
    bool writeSDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODWrite::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODWrite::Response> resp); // 20201117
    bool getRxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    bool getTxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
};

}

#endif
