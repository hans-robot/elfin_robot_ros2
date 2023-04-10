#ifndef ELFIN_ROS2_CONTROL_HW_INTERFACE
#define ELFIN_ROS2_CONTROL_HW_INTERFACE

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <elfin_ethercat_driver/elfin_ethercat_driver.h>
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

// using namespace ELFIN;
using hardware_interface::return_type;

namespace elfin_hardware_interface
{

typedef struct{
    std::string name;
    double reduction_ratio;
    double count_rad_factor;
    double count_rad_per_s_factor;
    double count_Nm_factor;
    int32_t count_zero;

    double axis_position_factor;
    double axis_torque_factor;

    double position;
    double velocity;
    double effort;

    double position_cmd;
    double velocity_cmd;
    double vel_ff_cmd;
    double effort_cmd;
}AxisInfo;

typedef struct{
    elfin_ethercat_driver::ElfinEtherCATClient* client_ptr;
    AxisInfo axis1;
    AxisInfo axis2;
}ModuleInfo;

class ElfinHWInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(ElfinHWInterface)

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type start() override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type stop() override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type read() override;

  // ELFIN_HARDWARE_INTERFACE_PUBLIC
  return_type write() override;

private:

  std::vector<std::string> elfin_driver_names_;
  elfin_ethercat_driver::EtherCatManager* em;
  std::vector<elfin_ethercat_driver::ElfinEtherCATDriver*> ethercat_drivers_;
  std::vector<ModuleInfo> module_infos_;

  rclcpp::Node::SharedPtr n_;

  rclcpp::Time read_update_time_;

  std::vector<bool> pre_switch_flags_;
  std::vector<boost::shared_ptr<boost::mutex> > pre_switch_mutex_ptrs_;
  
  
  bool isModuleMoving(int module_num);
  bool setGroupPosMode(const std::vector<int>& modules_no);
  double motion_threshold_ = 5e-5;

  bool pos_interface_claimed = false;
  bool pos_interface_running = false;
  bool first_pass_ = true;
  bool initialized_ = false;
  
  std::vector<double> new_commands_;
  std::vector<double> old_commands_;
};
} 

#endif
