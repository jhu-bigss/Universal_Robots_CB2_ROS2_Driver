// Copyright (c) 2022, BIGSS
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
// Author: Joshua Liu

#include <limits>
#include <vector>

#include "ur_cb2_robot_driver/hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ur_cb2_robot_driver
{
CallbackReturn URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // state interface
  ur_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // command interface
  ur_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // other hardware parameters
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  system_interface_initialized_ = 0.0;
  time_last_cmd_send_ = rclcpp::Clock().now();

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
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
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    // position interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ur_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &ur_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &ur_joint_efforts_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("system_interface", "initialized", &system_interface_initialized_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    // position interfaces
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ur_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &ur_velocity_commands_[i]));
  }

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));

  return command_interfaces;
}

CallbackReturn URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  std::string robot_name = info_.hardware_parameters["name"];
  std::string ur_type = info_.hardware_parameters["ur_type"];
  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Path to the urscript code that will be sent to the robot
  std::string script_filename = info_.hardware_parameters["script_filename"];

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");

  // Set the UR type
  this->SetURType(ur_type);

  // Connect to UR robot server
  this->ConnectToUR(robot_ip);
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Is connected to UR? = %s", isConnectedToUR ? "Yes" : "No");
  if (!isConnectedToUR){
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Could not connect to Robot!");
    return CallbackReturn::ERROR;
  }
  else{
    // Send UR script to robot controller
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Sending UR Program...");
    this->ProgramFromFile(script_filename);
  }

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn URPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  // Anything to clean up?

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type URPositionHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Parse status from the UR
  // update the state using the default state updates the UR gives over the specified UR socket
  ur_interface.readFromSocket(urSocket);
  packet_read_ = true;
  
  // joint positions, velocities, and jacobians
  for (uint i = 0; i < info_.joints.size(); i++) {
    UniversalRobot::SingleJoint cur_joint_ = ur_interface.jointData.jd.joint[i];
    ur_joint_positions_[i] = ur_interface.unionValue(cur_joint_.q_actual);
    ur_joint_velocities_[i] = ur_interface.unionValue(cur_joint_.qd_actual);
    ur_joint_efforts_[i] = ur_interface.unionValue(cur_joint_.I_actual);
    // UniversalRobot::JointMode joint_mode_[i] = static_cast<UniversalRobot::JointMode>(cur_joint_.jointMode);
  }
    ur_jacobians_ = ur_interface.getGeoJacobian();

    ur_tcp_pose_[0] = ur_interface.unionValue(ur_interface.cartesianInfo.info.x);
    ur_tcp_pose_[1] = ur_interface.unionValue(ur_interface.cartesianInfo.info.y);
    ur_tcp_pose_[2] = ur_interface.unionValue(ur_interface.cartesianInfo.info.z);
    ur_tcp_pose_[3] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Rx);
    ur_tcp_pose_[4] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Ry);
    ur_tcp_pose_[5] = ur_interface.unionValue(ur_interface.cartesianInfo.info.Rz);
    extractToolPose();

  // other robot states data
  robot_mode_data_ = ur_interface.robotMode.rmd;
  robot_mode_ = static_cast<UniversalRobot::RobotMode>(robot_mode_data_.state.robotMode);
  target_speed_fraction_ = ur_interface.unionValue(robot_mode_data_.targetSpeedFraction);

  if (first_pass_ && !initialized_)
  {
    // initialize commands
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;
    ur_velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    target_speed_fraction_cmd_ = NO_NEW_CMD_;
    resend_robot_program_cmd_ = NO_NEW_CMD_;
    initialized_ = true;
    system_interface_initialized_ = 1.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // If there is no interpreting program running on the robot, we do not want to send anything.
  time_now_ = rclcpp::Clock().now();
  rclcpp::Duration time_since_last_send_ = time_now_ - time_last_cmd_send_;

  if (isConnectedToProg && robot_mode_data_.state.isProgramRunning && packet_read_ && time_since_last_send_ >= rclcpp::Duration(0, 8000000))
  {
    if (position_controller_running_ && (ur_position_commands_ != ur_position_commands_old_))
    {
      this->SetPositionJoint(ur_position_commands_);
      ur_position_commands_old_ = ur_position_commands_;
    }
    else if (velocity_controller_running_)
    {
      this->SetVelocityJoint(ur_velocity_commands_);
    }
    else
    {
      // Do something to keep it alive
    }
    time_last_cmd_send_ = time_now_;
    packet_read_ = false;
  }

  return hardware_interface::return_type::OK;
}

void URPositionHardwareInterface::extractToolPose()
{
  // imported from ROS1 driver hardware_interface.cpp#L911-L928
  double tcp_angle =
      std::sqrt(std::pow(ur_tcp_pose_[3], 2) + std::pow(ur_tcp_pose_[4], 2) + std::pow(ur_tcp_pose_[5], 2));

  tf2::Vector3 rotation_vec(ur_tcp_pose_[3], ur_tcp_pose_[4], ur_tcp_pose_[5]);
  tf2::Quaternion rotation;
  if (tcp_angle > 1e-16) {
    rotation.setRotation(rotation_vec.normalized(), tcp_angle);
  } else {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
  }
  tcp_transform_.transform.translation.x = ur_tcp_pose_[0];
  tcp_transform_.transform.translation.y = ur_tcp_pose_[1];
  tcp_transform_.transform.translation.z = ur_tcp_pose_[2];

  tcp_transform_.transform.rotation = tf2::toMsg(rotation);
}

hardware_interface::return_type URPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
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

hardware_interface::return_type URPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
    position_controller_running_ = false;
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    ur_position_commands_ = ur_position_commands_old_ = ur_joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
    position_controller_running_ = false;
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}
}  // namespace ur_cb2_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_cb2_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
