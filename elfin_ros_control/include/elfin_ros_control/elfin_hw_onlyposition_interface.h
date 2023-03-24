#ifndef ELFIN_ROS2_CONTROL_HW_POSITIONONLY_INTERFACE
#define ELFIN_ROS2_CONTROL_HW_POSITIONONLY_INTERFACE

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <elfin_ethercat_driver/elfin_ethercat_driver.h>
#include <elfin_ethercat_driver/elfin_ethercat_manager.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string>
#include <memory>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager/controller_manager.hpp>
#include <std_msgs/msg/float64.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "visibility_control.h"

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace elfin_hardware_interface
{
class ElfinHWInterface_PositoinOnly
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElfinHWInterface_PositoinOnly)

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  rclcpp::Node::SharedPtr n_;
  
  bool first_pass_ = true;
  bool initialized_ = false;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::vector<double> elfin_joint_positions_;
  std::vector<double> elfin_joint_velocities_;
  std::vector<double> elfin_joint_efforts_;
  std::vector<double> elfin_ft_sensor_measurements_;
  std::vector<double> elfin_tcp_pose_;
  std::vector<double> elfin_position_commands_;
  std::vector<double> elfin_position_commands_old_;
  std::vector<double> elfin_velocity_commands_;

};

}
#endif