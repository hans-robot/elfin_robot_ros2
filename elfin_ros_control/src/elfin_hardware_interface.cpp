#include <elfin_ros_control/elfin_hw_interface.h>
#include <elfin_ros_control/visibility_control.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <hardware_interface/handle.hpp>


namespace elfin_hardware_interface{

  bool ElfinHWInterface::isModuleMoving(int module_num)
  {
    std::vector<double> previous_pos;
    std::vector<double> last_pos;

    previous_pos.resize(2);
    last_pos.resize(2);

    int32_t count1, count2;

    module_infos_[module_num].client_ptr->getActPosCounts(count1,count2);
    previous_pos[0] = count1/module_infos_[module_num].axis1.count_rad_factor;
    previous_pos[1] = count2/module_infos_[module_num].axis2.count_rad_factor;

    usleep(10000);

    module_infos_[module_num].client_ptr->getActPosCounts(count1, count2);
    last_pos[0] = count1/module_infos_[module_num].axis1.count_rad_factor;
    last_pos[1] = count2/module_infos_[module_num].axis2.count_rad_factor;

    for(unsigned int i=0;i<previous_pos.size();i++)
    {
      if(fabs(last_pos[i] - previous_pos[i])>motion_threshold_)
      {
        return true;
      }
    }
    return false;
  }

  bool ElfinHWInterface::setGroupPosMode(const std::vector<int> &module_no)
  {
    for(unsigned int j=0;j<module_no.size();j++)
    {
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
      pre_switch_flags_[module_no[j]] = true;
      pre_switch_flags_lock.unlock();
    }

    usleep(10000);

    for(unsigned int j=0;j<module_no.size();j++)
    {
      module_infos_[module_no[j]].client_ptr->setPosMode();
    }

    usleep(10000);

    for(unsigned int j=0;j<module_no.size();j++)
    {
      if(!module_infos_[module_no[j]].client_ptr->inPosMode())
        {
            RCLCPP_ERROR(n_->get_logger(), "module[%i]: set position mode failed", module_no[j]);
            for(unsigned int k=0; k<module_no.size(); k++)
            {
                boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
                pre_switch_flags_[module_no[k]]=false;
                pre_switch_flags_lock.unlock();
            }

            return false;
        }
    }
  }

  return_type ElfinHWInterface::configure(const hardware_interface::HardwareInfo & info)
  {
    n_ = rclcpp::Node::make_shared("elfin_hw");
    std::vector<std::string> elfin_driver_names_default;
    std::string ethercat_name_default = "eth0";
    elfin_driver_names_default.resize(1);
    elfin_driver_names_default[0] = "elfin";
    n_->declare_parameter("elfin_ethercat_drivers",std::vector<std::string>({"elfin"}));
    n_->get_parameter_or("elfin_ethercat_drivers",elfin_driver_names_, elfin_driver_names_default);
    std::string ethernet_name;
    n_->declare_parameter("elfin_ethernet_name","eth0");
    n_->get_parameter_or("elfin_ethernet_name", ethernet_name,ethercat_name_default);
    ethercat_drivers_.clear();
    ethercat_drivers_.resize(elfin_driver_names_.size());

    new_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    old_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    em = new elfin_ethercat_driver::EtherCatManager(ethernet_name);
    for(unsigned int i=0;i<ethercat_drivers_.size();i++)
    {
      ethercat_drivers_[i] = new elfin_ethercat_driver::ElfinEtherCATDriver(em,elfin_driver_names_[i],n_);
    }
    module_infos_.clear();

    for (size_t i =0;i<ethercat_drivers_.size();i++)
    {
      for(size_t j =0;j<ethercat_drivers_[i]->getEtherCATClientNumber();j++)
      {
        ModuleInfo module_info_tmp;
        module_info_tmp.client_ptr = ethercat_drivers_[i]->getEtherCATClientPtr(j);
        
        module_info_tmp.axis1.name = ethercat_drivers_[i]->getJointName(2*j);
        module_info_tmp.axis1.reduction_ratio = ethercat_drivers_[i]->getReductionRatio(2*j);
        module_info_tmp.axis1.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j);
        module_info_tmp.axis1.count_zero = ethercat_drivers_[i]->getCountZero(2*j);
        module_info_tmp.axis1.axis_torque_factor = ethercat_drivers_[i]->getAxisTorqueFactor(2*j);

        module_info_tmp.axis2.name = ethercat_drivers_[i]->getJointName(2*j+1);
        module_info_tmp.axis2.reduction_ratio = ethercat_drivers_[i]->getReductionRatio(2*j+1);
        module_info_tmp.axis2.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j+1);
        module_info_tmp.axis2.count_zero =ethercat_drivers_[i]->getCountZero(2*j+1);
        module_info_tmp.axis2.axis_torque_factor = ethercat_drivers_[i]->getAxisTorqueFactor(2*j+1);
        module_infos_.push_back(module_info_tmp);
      }
    }

    for(size_t i =0;i<module_infos_.size();i++)
    {
      module_infos_[i].axis1.count_rad_factor = module_infos_[i].axis1.reduction_ratio*module_infos_[i].axis1.axis_position_factor/(2*M_PI);
      module_infos_[i].axis1.count_rad_per_s_factor = module_infos_[i].axis1.count_rad_factor/750.3;
      module_infos_[i].axis1.count_Nm_factor = module_infos_[i].axis1.axis_torque_factor / module_infos_[i].axis1.reduction_ratio;

      module_infos_[i].axis2.count_rad_factor = module_infos_[i].axis2.reduction_ratio*module_infos_[i].axis2.axis_position_factor/(2*M_PI);
      module_infos_[i].axis2.count_rad_per_s_factor = module_infos_[i].axis2.count_rad_factor/750.3;
      module_infos_[i].axis2.count_Nm_factor = module_infos_[i].axis2.axis_torque_factor / module_infos_[i].axis2.reduction_ratio;
    }
    
    for(size_t i=0; i<module_infos_.size(); i++)
    {
        int32_t pos_count1=module_infos_[i].client_ptr->getAxis1PosCnt();
        double position_tmp_1=(pos_count1-module_infos_[i].axis1.count_zero)/module_infos_[i].axis1.count_rad_factor;
        if(position_tmp_1>=M_PI)
        {
            module_infos_[i].axis1.count_zero+=module_infos_[i].axis1.count_rad_factor*2*M_PI;
        }
        else if(position_tmp_1<-1*M_PI)
        {
            module_infos_[i].axis1.count_zero-=module_infos_[i].axis1.count_rad_factor*2*M_PI;
        }
        module_infos_[i].axis1.position=-1*(pos_count1-module_infos_[i].axis1.count_zero)/module_infos_[i].axis1.count_rad_factor;

        int32_t pos_count2=module_infos_[i].client_ptr->getAxis2PosCnt();
        double position_tmp_2=(pos_count2-module_infos_[i].axis2.count_zero)/module_infos_[i].axis2.count_rad_factor;
        if(position_tmp_2>=M_PI)
        {
            module_infos_[i].axis2.count_zero+=module_infos_[i].axis2.count_rad_factor*2*M_PI;
        }
        else if(position_tmp_2<-1*M_PI)
        {
            module_infos_[i].axis2.count_zero-=module_infos_[i].axis2.count_rad_factor*2*M_PI;
        }
        module_infos_[i].axis2.position=-1*(pos_count2-module_infos_[i].axis2.count_zero)/module_infos_[i].axis2.count_rad_factor;
    }
    
    pre_switch_flags_.resize(module_infos_.size());
    for(unsigned int i=0;i<pre_switch_flags_.size();i++)
    {
      pre_switch_flags_[i] = false;
    }

    pre_switch_mutex_ptrs_.resize(module_infos_.size());
    for(unsigned int i=0;i<pre_switch_mutex_ptrs_.size();i++)
    {
      pre_switch_mutex_ptrs_[i] = boost::shared_ptr<boost::mutex>(new boost::mutex);
    }
    // for(unsigned int i=0;i<ethercat_drivers_.size();i++)
    // {
    //   ethercat_drivers_[i]->enableRobot_test();
    // }
    return return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> ElfinHWInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0;i<module_infos_.size();i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis1.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis1.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis2.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis2.position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis1.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis1.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis2.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis2.velocity));

      //state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis1.name, hardware_interface::HW_IF_EFFORT, &module_infos_[i].axis1.effort));
      //state_interfaces.emplace_back(hardware_interface::StateInterface(module_infos_[i].axis2.name, hardware_interface::HW_IF_EFFORT, &module_infos_[i].axis2.effort));

    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ElfinHWInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(module_infos_.size());
    for (unsigned int i = 0; i < module_infos_.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis1.position_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_POSITION, &module_infos_[i].axis2.position_cmd));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis1.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis1.velocity_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        module_infos_[i].axis2.name, hardware_interface::HW_IF_VELOCITY, &module_infos_[i].axis2.velocity_cmd));
    }
    return command_interfaces;
  }

  return_type ElfinHWInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
  {
    auto cur_interface = [](const std::string& interface){
        return interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
    };
    int64_t num_stop_cur_interfaces = 
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), cur_interface);
    if(num_stop_cur_interfaces == 6){
      pos_interface_claimed = false;
    }else if(num_stop_cur_interfaces != 0){
      RCLCPP_FATAL(n_->get_logger(),"Expected %d pos interfaces to stop, but god %d instead.",6,num_stop_cur_interfaces);
      std::string error_string = "Invalid number of pos interfaces to stop,Expected";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }

    int64_t num_start_cur_interfaces = 
      std::count_if(start_interfaces.begin(), start_interfaces.end(),cur_interface);
    if(num_start_cur_interfaces == 6){
      pos_interface_claimed = true;
    }else if(num_start_cur_interfaces != 0){
      RCLCPP_FATAL(n_->get_logger(),"Expected %d pos interfaces to start, but god %d instead.",6,num_start_cur_interfaces);
      std::string error_string = "Invalid manmagerf of pos interfaces top stop. Execeptd";
      error_string += std::to_string(6);
      throw std::invalid_argument(error_string);
    }
    return return_type::OK;
  }

  return_type ElfinHWInterface::perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&)
  {
    if(pos_interface_claimed &&  !pos_interface_running)
    {
      pos_interface_running = true;
    }else if(pos_interface_running && !pos_interface_claimed)
    {
      pos_interface_running = false;
    }
    return return_type::OK;
  }

  return_type ElfinHWInterface::start()
  {
    read();
    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO(n_->get_logger(),"Started");
    return return_type::OK;
  }

  return_type ElfinHWInterface::stop()
  {
    RCLCPP_INFO(n_->get_logger(),"trying to Stop");
    status_ = hardware_interface::status::STOPPED;
    for(unsigned int i=0;i<ethercat_drivers_.size();i++)
    {
      if(ethercat_drivers_[i]!=NULL)
      {
        delete ethercat_drivers_[i];
      }
    }
    RCLCPP_INFO(n_->get_logger(), "Stopped");
    return return_type::OK;
  }

  return_type ElfinHWInterface::read()
  {
    rclcpp::spin_some(n_);
    for(size_t i=0;i<module_infos_.size();i++)
    {
      int32_t pos_count1 = module_infos_[i].client_ptr->getAxis1PosCnt();
      int16_t vel_count1 = module_infos_[i].client_ptr->getAxis1VelCnt();
      int16_t trq_count1 = module_infos_[i].client_ptr->getAxis1TrqCnt();
      int32_t pos_count_diff_1 = pos_count1 - module_infos_[i].axis1.count_zero;

      double position_tmp1 = -1*pos_count_diff_1/module_infos_[i].axis1.count_rad_factor;
      module_infos_[i].axis1.position = position_tmp1;
      module_infos_[i].axis1.velocity = -1*vel_count1/module_infos_[i].axis1.count_rad_per_s_factor;
      module_infos_[i].axis1.effort = -1*trq_count1/module_infos_[i].axis1.count_Nm_factor;
      

      int32_t pos_count2 = module_infos_[i].client_ptr->getAxis2PosCnt();
      int16_t vel_count2 = module_infos_[i].client_ptr->getAxis2VelCnt();
      int16_t trq_count2 = module_infos_[i].client_ptr->getAxis2TrqCnt();
      int32_t pos_count_diff_2 = pos_count2 - module_infos_[i].axis2.count_zero;
      
      double position_tmp2 = -1*pos_count_diff_2/module_infos_[i].axis2.count_rad_factor;
      module_infos_[i].axis2.position = position_tmp2;
      module_infos_[i].axis2.velocity = -1*vel_count2/module_infos_[i].axis2.count_rad_per_s_factor;
      module_infos_[i].axis2.effort = -1*trq_count2/module_infos_[i].axis2.count_Nm_factor;

      if (first_pass_ && !initialized_) {
        module_infos_[i].axis1.position_cmd = module_infos_[i].axis1.position;
        module_infos_[i].axis2.position_cmd = module_infos_[i].axis2.position;

        new_commands_[2*i] = pos_count1;
        new_commands_[2*i+1] = pos_count2;

        old_commands_[2*i] = pos_count1;
        old_commands_[2*i+1] = pos_count2;
      }
    }
    initialized_ = true;
    
    return return_type::OK;
  }

  return_type ElfinHWInterface::write()
  {
    bool robot_move = false;
    for(size_t i =0;i<module_infos_.size();i++)
    {
      if(!module_infos_[i].client_ptr->inPosBasedMode())
      {
        module_infos_[i].axis1.position_cmd = module_infos_[i].axis1.position;
        module_infos_[i].axis2.position_cmd = module_infos_[i].axis2.position;
      }
      double position_cmd_count1 = -1*module_infos_[i].axis1.position_cmd * module_infos_[i].axis1.count_rad_factor + module_infos_[i].axis1.count_zero;
      double position_cmd_count2 = -1*module_infos_[i].axis2.position_cmd * module_infos_[i].axis2.count_rad_factor + module_infos_[i].axis2.count_zero;

      new_commands_[2*i] = position_cmd_count1;
      new_commands_[2*i+1] = position_cmd_count2;

      if(fabs((new_commands_[2*i]- module_infos_[i].axis1.count_zero) - (old_commands_[2*i]- module_infos_[i].axis1.count_zero))/module_infos_[i].axis1.count_rad_factor>motion_threshold_)
      {  
        module_infos_[i].client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
        robot_move = true;
        old_commands_[2*i] = new_commands_[2*i];
      }
      
      if(fabs((new_commands_[2*i+1]- module_infos_[i].axis2.count_zero) - (old_commands_[2*i+1]- module_infos_[i].axis2.count_zero))/module_infos_[i].axis2.count_rad_factor>motion_threshold_)
      {
         module_infos_[i].client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));
         robot_move = true;
         old_commands_[2*i+1] = new_commands_[2*i+1];
      }
  
      bool is_preparing_switch;
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
      is_preparing_switch = pre_switch_flags_[i];
      pre_switch_flags_lock.unlock();

      if(!is_preparing_switch)
      {
        double vel_ff_cmd_count1 = -1 * module_infos_[i].axis1.vel_ff_cmd * module_infos_[i].axis1.count_rad_per_s_factor /16.0;
        double vel_ff_cmd_count2 = -1 * module_infos_[i].axis2.vel_ff_cmd * module_infos_[i].axis2.count_rad_per_s_factor /16.0;

        //module_infos_[i].client_ptr->setAxis1VelFFCnt(int16_t(vel_ff_cmd_count1));
        //module_infos_[i].client_ptr->setAxis2VelFFCnt(int16_t(vel_ff_cmd_count2));

        double torque_cmd_count1 = -1*module_infos_[i].axis1.effort_cmd * module_infos_[i].axis1.count_Nm_factor;
        double torque_cmd_count2 = -1*module_infos_[i].axis2.effort_cmd * module_infos_[i].axis2.count_Nm_factor;

        //module_infos_[i].client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
        //module_infos_[i].client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2));
      }
    }
    if(!robot_move){
      ethercat_drivers_[0]->ethercat_io_clients_[0]->pub_io_state();
    }
    return return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(elfin_hardware_interface::ElfinHWInterface,
                       hardware_interface::SystemInterface)