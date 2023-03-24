#include <elfin_ros_control/elfin_hw_onlyposition_interface.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>

namespace elfin_hardware_interface
{

hardware_interface::return_type ElfinHWInterface_PositoinOnly::configure(
  const hardware_interface::HardwareInfo & info)
{
   if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  elfin_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  elfin_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  elfin_position_commands_ = elfin_position_commands_old_ = elfin_joint_positions_;
  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ElfinHWInterface_PositoinOnly::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ElfinHWInterface_PositoinOnly::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type ElfinHWInterface_PositoinOnly::start()
{
  n_ = rclcpp::Node::make_shared("elfin_hw");
  read();
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Starting ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // set some default values when starting the first time
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
    else
    {
      hw_commands_[i] = hw_states_[i];
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElfinHWInterface_PositoinOnly::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Stopping ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElfinHWInterface_PositoinOnly::read()
{
  rclcpp::spin_some(n_);
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]);// / hw_slowdown_;
    //RCLCPP_INFO(
    //  rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Got state %.5f for joint %d!",
    //  hw_states_[i], i);
  }
  if (first_pass_ && !initialized_) {
      // initialize commands
      elfin_position_commands_ = elfin_position_commands_old_ = hw_states_;
      initialized_ = true;
      for (uint i = 0; i < hw_states_.size(); i++)
        {
            //RCLCPP_INFO(
            //    rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Got state %.5f for joint %d!",
            //hw_states_[i], i);
        }
    }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElfinHWInterface_PositoinOnly::write()
{
    bool new_data_available = false;
    std::function<double(double, double)> substractor = [](double a, double b) { return std::abs(a - b); };
    std::vector<double> pos_diff;
    pos_diff.resize(hw_commands_.size());
    std::transform(hw_commands_.begin(), hw_commands_.end(), elfin_position_commands_old_.begin(),
                   pos_diff.begin(), substractor);

    double pos_diff_sum = 0.0;
    std::for_each(pos_diff.begin(), pos_diff.end(), [&pos_diff_sum](double a) { return pos_diff_sum += a; });

    if (pos_diff_sum != 0.0) {
      new_data_available = true;
    }
    if(new_data_available){
        for (uint i = 0; i < hw_commands_.size(); i++)
        {
            //RCLCPP_INFO(
            //rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Got command %.5f for joint %d!",
            //hw_commands_[i], i);
        }
        elfin_position_commands_old_ = hw_commands_;
    }
  elfin_position_commands_old_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  elfin_hardware_interface::ElfinHWInterface_PositoinOnly, hardware_interface::SystemInterface)
