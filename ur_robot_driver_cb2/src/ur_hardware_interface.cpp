#include <pluginlib/class_list_macros.hpp>
#include <ur_robot_driver_cb2/ur_hardware_interface.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ur_robot_driver_cb2
{
  
URPositionHardwareInterface::URPositionHardwareInterface()
{
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
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

  max_payload_ = 1.0;

  // Set the following gpio variable to 0.0 for now
  // these variables are necessary to match ur_description/urdf/ur.ros2_control.xacro
  actual_dig_out_bits_ = {0.0};
  actual_dig_in_bits_ = {0.0};
  analog_io_types_ = {0.0};
  robot_status_bits_ = {0.0};
  safety_status_bits_ = {0.0};
  tool_analog_input_types_ = {0.0};
  tool_analog_input_ = {0.0};
  standard_analog_input_ = {0.0};
  standard_analog_output_ = {0.0};
  tool_output_voltage_ = 0.0;
  robot_mode_ = 0.0;
  safety_mode_ = 0.0;
  tool_mode_ = 0.0;
  tool_output_current_ = 0.0;
  tool_temperature_ = 0.0;
  system_interface_initialized_ = 0.0;
  robot_program_running_ = 0.0;

  io_async_success_ = 0.0;
  target_speed_fraction_cmd_ = 0.0;
  scaling_async_success_ = 0.0;
  resend_robot_program_cmd_ = 0.0;
  resend_robot_program_async_success_ = 0.0;
  hand_back_control_cmd_ = 0.0;
  hand_back_control_async_success_ = 0.0;
  payload_mass_ = 0.0;
  payload_center_of_gravity_ = {0.0, 0.0, 0.0};
  payload_async_success_ = 0.0;
  standard_dig_out_bits_cmd_ = {0.0};
  standard_analog_output_cmd_ = {0.0};
  tool_voltage_cmd_ = 0.0;
  zero_ftsensor_cmd_ = 0.0;
  zero_ftsensor_async_success_ = 0.0;

  double max_acceleration=15;
  max_velocity_=10;
  acceleration_coeff_=2;
  max_vel_change_=max_acceleration/125;
  curr2torque_.resize(6);
    
  
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "joints num : %ld",info_.joints.size());
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
  }

  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "speed_scaling", "speed_scaling_factor",
                                                                   &speed_scaling_combined_));

  for (auto& sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "sensor name %s",sensor.name.c_str());
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
                                                                       &urcl_ft_sensor_measurements_[j]));
    }
  }

  for (size_t i = 0; i < 18; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_[i]));
  }

  for (size_t i = 0; i < 11; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits_[i]));
  }

  for (size_t i = 0; i < 4; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized",
                                                                   &system_interface_initialized_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
  }

  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "gpio", "io_async_success", &io_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "hand_back_control", "hand_back_control_cmd", &hand_back_control_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "hand_back_control", "hand_back_control_async_success", &hand_back_control_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "payload", "mass", &payload_mass_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.x", &payload_center_of_gravity_[0]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.y", &payload_center_of_gravity_[1]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.z", &payload_center_of_gravity_[2]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "payload_async_success", &payload_async_success_));

  for (size_t i = 0; i < 18; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
  }

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "gpio", "tool_voltage_cmd", &tool_voltage_cmd_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "zero_ftsensor", "zero_ftsensor_cmd", &zero_ftsensor_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "zero_ftsensor", "zero_ftsensor_async_success", &zero_ftsensor_async_success_));

  return command_interfaces;
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Configuring UR driver");

  std::string robot_ip   = info_.hardware_parameters["robot_ip"];
  int reverse_port       = stoi(info_.hardware_parameters["reverse_port"]);

  ur_driver_.reset(new ur_robot_driver_cb2::UrDriver(rt_msg_cond_, msg_cond_, robot_ip, reverse_port,12,0, max_payload_));
  ur_driver_->setJointNames(joint_names_);
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting UR driver... please wait...");

  if (!ur_driver_->start())
  {
    RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "Error creating connection");
    return CallbackReturn::ERROR;
  }

  std::vector<double> pos_init;
  for (int i=0;i<100;i++)
  {
    pos_init  = ur_driver_->rt_interface_->robot_state_->getQActual();
    for (unsigned int idx=0;idx<6;idx++)
      {
        // RCLCPP_WARN_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "Initial Position joint " << idx << ": " << pos_init.at(idx));
        urcl_position_commands_.at(idx)=pos_init.at(idx);
        urcl_position_commands_old_.at(idx)=pos_init.at(idx);
      }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn URPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "stopping UR driver");
  #ifdef LOG
  out.close();
  #endif
  ur_driver_->halt();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type URPositionHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
  std::vector<double> pos, vel, eff, tcp;

  pos             = ur_driver_->rt_interface_->robot_state_->getQActual();
  vel             = ur_driver_->rt_interface_->robot_state_->getQdActual();
  eff             = ur_driver_->rt_interface_->robot_state_->getIActual();
  tcp_force_     = ur_driver_->rt_interface_->robot_state_->getTcpForce();

  #ifdef LOG
  auto p1 = std::chrono::system_clock::now();
  out << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << "," << pos[4] << "," << pos[5] << "," << std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count() << "\n";
  #endif

  for (unsigned int idx=0;idx<6;idx++)
  {
    urcl_joint_positions_.at(idx)=pos.at(idx);
    urcl_joint_velocities_.at(idx)=vel.at(idx);
    urcl_joint_efforts_.at(idx)=eff.at(idx);//*curr2torque_.at(idx);
    urcl_ft_sensor_measurements_.at(idx)=tcp_force_.at(idx);
  }

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type URPositionHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{

  if (position_controller_running_) {
    ur_driver_->setPosition(urcl_position_commands_.at(0),urcl_position_commands_.at(1),urcl_position_commands_.at(2)
                          ,urcl_position_commands_.at(3),urcl_position_commands_.at(4),urcl_position_commands_.at(5)
                          // ,0.008,0.1,300);
                          ,0.016,0.05,700);
    // std::cout << "MEGAOUT" << urcl_position_commands_.at(0) <<" " << urcl_position_commands_.at(1)<<" " << urcl_position_commands_.at(2)<<" " << urcl_position_commands_.at(3)
                          //  <<" " << urcl_position_commands_.at(4)<<" " << urcl_position_commands_.at(5)<<" " <<"\n" << std::flush;	

  } else if (velocity_controller_running_) {
    // RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "SPEED not yet tested");
    ur_driver_->setSpeed(urcl_velocity_commands_.at(0), urcl_velocity_commands_.at(1), urcl_velocity_commands_.at(2)
                      , urcl_velocity_commands_.at(3), urcl_velocity_commands_.at(4), urcl_velocity_commands_.at(5)
                      ,  0.5);
    // std::cout << "MEGAOUT" << urcl_velocity_commands_.at(0) <<" " << urcl_velocity_commands_.at(1)<<" " << urcl_velocity_commands_.at(2)<<" " << urcl_velocity_commands_.at(3)
                          //  <<" " << urcl_velocity_commands_.at(4)<<" " << urcl_velocity_commands_.at(5)<<" " <<"\n" << std::flush;	

    // else {
    //   ur_driver_->writeKeepalive();
    // }
    //TODO

  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
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

hardware_interface::return_type URPositionHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
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

PLUGINLIB_EXPORT_CLASS(ur_robot_driver_cb2::URPositionHardwareInterface, hardware_interface::SystemInterface)
