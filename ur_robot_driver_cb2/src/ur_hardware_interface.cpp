#include <pluginlib/class_list_macros.hpp>
#include <ur_robot_driver_cb2/ur_hardware_interface.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cb2_hw
{
  
UrRobotHW::UrRobotHW()
{
}

hardware_interface::CallbackReturn UrRobotHW::on_init(const hardware_interface::HardwareInfo& system_info)
{

  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  #ifdef LOG
  out.open ("out.txt");
  #endif
  info_ = system_info;

  // initialize
  urcl_joint_positions_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_joint_velocities_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_joint_efforts_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_ft_sensor_measurements_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_tcp_pose_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_position_commands_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_position_commands_old_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  urcl_velocity_commands_ =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ;
  stop_modes_ = { StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
                  StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE };
  start_modes_ = {};
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  // runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  // system_interface_initialized_ = 0.0;
  
  // torque_pub_ = rclcpp::node_interfaces::get_node_base_interface().create_publisher<geometry_msgs::msg::Wrench>("wrench", 1);

  //Create UrDriver;

  double max_payload=1;
  double max_acceleration=15;
  m_max_velocity=10;
  m_acceleration_coeff=2;
  m_max_vel_change=max_acceleration/125;
  m_curr2torque.resize(6);
    
  
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    m_joint_names.push_back(joint.name);
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UrRobotHW::export_state_interfaces()
{
  RCLCPP_WARN(rclcpp::get_logger("UrRobotHW"), "joints num : %ld",info_.joints.size());
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
  }

  // state_interfaces.emplace_back(
  //     hardware_interface::StateInterface("speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

  for (auto& sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      RCLCPP_WARN(rclcpp::get_logger("UrRobotHW"), "sensor name %s",sensor.name.c_str());
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
                                                                       &urcl_ft_sensor_measurements_[j]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UrRobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
  }

  //TODO payload
  return command_interfaces;

}

hardware_interface::CallbackReturn UrRobotHW::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "Starting ...please wait...");

  std::string robot_ip   = info_.hardware_parameters["robot_ip"];
  int reverse_port       = stoi(info_.hardware_parameters["reverse_port"]);

  double max_payload     = 1.0;

  m_driver.reset(new cb2_hw::UrDriver(m_rt_msg_cond, m_msg_cond, robot_ip, reverse_port,12,0,max_payload));
  m_driver->setJointNames(m_joint_names);
  
  if (!m_driver->start())
  {
    RCLCPP_WARN(rclcpp::get_logger("UrRobotHW"), "Error creating connection");
    return CallbackReturn::ERROR;
  }

  std::vector<double> pos_init;
  for (int i=0;i<100;i++)
  {
    pos_init  = m_driver->rt_interface_->robot_state_->getQActual();
    for (unsigned int idx=0;idx<6;idx++)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("UrRobotHW"), "Initial Position joint " << idx << ": " << pos_init.at(idx));
        urcl_position_commands_.at(idx)=pos_init.at(idx);
        urcl_position_commands_old_.at(idx)=pos_init.at(idx);
      }
  }

  return CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn UrRobotHW::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_WARN(rclcpp::get_logger("UrRobotHW"), "stopping driver");
  #ifdef LOG
  out.close();
  #endif
  m_driver->halt();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type UrRobotHW::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
  std::vector<double> pos, vel, eff, tcp;

  pos             = m_driver->rt_interface_->robot_state_->getQActual();
  vel             = m_driver->rt_interface_->robot_state_->getQdActual();
  eff             = m_driver->rt_interface_->robot_state_->getIActual();
  m_tcp_force     = m_driver->rt_interface_->robot_state_->getTcpForce();

  #ifdef LOG
  auto p1 = std::chrono::system_clock::now();
  out << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << "," << pos[4] << "," << pos[5] << "," << std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count() << "\n";
  #endif

  for (unsigned int idx=0;idx<6;idx++)
  {
    urcl_joint_positions_.at(idx)=pos.at(idx);
    urcl_joint_velocities_.at(idx)=vel.at(idx);
    urcl_joint_efforts_.at(idx)=eff.at(idx);//*m_curr2torque.at(idx);
    urcl_ft_sensor_measurements_.at(idx)=m_tcp_force.at(idx);
  }

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type UrRobotHW::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{

  if (position_controller_running_) {
    m_driver->setPosition(urcl_position_commands_.at(0),urcl_position_commands_.at(1),urcl_position_commands_.at(2)
                          ,urcl_position_commands_.at(3),urcl_position_commands_.at(4),urcl_position_commands_.at(5)
                          // ,0.008,0.1,300);
                          ,0.016,0.05,700);
    // std::cout << "MEGAOUT" << urcl_position_commands_.at(0) <<" " << urcl_position_commands_.at(1)<<" " << urcl_position_commands_.at(2)<<" " << urcl_position_commands_.at(3)
                          //  <<" " << urcl_position_commands_.at(4)<<" " << urcl_position_commands_.at(5)<<" " <<"\n" << std::flush;	

  } else if (velocity_controller_running_) {
    // RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"), "SPEED not yet tested");
    m_driver->setSpeed(urcl_velocity_commands_.at(0), urcl_velocity_commands_.at(1), urcl_velocity_commands_.at(2)
                      , urcl_velocity_commands_.at(3), urcl_velocity_commands_.at(4), urcl_velocity_commands_.at(5)
                      ,  0.5);
    // std::cout << "MEGAOUT" << urcl_velocity_commands_.at(0) <<" " << urcl_velocity_commands_.at(1)<<" " << urcl_velocity_commands_.at(2)<<" " << urcl_velocity_commands_.at(3)
                          //  <<" " << urcl_velocity_commands_.at(4)<<" " << urcl_velocity_commands_.at(5)<<" " <<"\n" << std::flush;	

    // else {
    //   m_driver->writeKeepalive();
    // }
    //TODO

  }

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type UrRobotHW::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  start_modes_.clear();
  stop_modes_.clear();

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
      }
    }
  }
  // set new mode to all interfaces at the same time
  if (start_modes_.size() != 0 && start_modes_.size() != 6) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // all start interfaces must be the same - can't mix position and velocity control
  if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_.push_back(StoppingInterface::STOP_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
      }
    }
  }
  // stop all interfaces at the same time
  if (stop_modes_.size() != 0 &&
      (stop_modes_.size() != 6 || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  controllers_initialized_ = true;
  return ret_val;
}

hardware_interface::return_type UrRobotHW::perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
    position_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
    position_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}



}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cb2_hw::UrRobotHW, hardware_interface::SystemInterface)

