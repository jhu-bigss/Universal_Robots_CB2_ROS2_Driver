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

#include <limits>
#include <vector>

#include "ur_robot_driver/hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ur_robot_driver/ur/bigss_debug_util.h"

namespace ur_robot_driver
{
CallbackReturn URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // state interface
  ur_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // command interface
  ur_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // other hardware parameters
  // hw_port_ = std::stoul(info_.hardware_parameters["port"]);
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  runtime_state_ = RuntimeState::STOPPED;
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  async_thread_shutdown_ = false;
  system_interface_initialized_ = 0.0;

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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ur_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &ur_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &ur_efforts_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

  for (size_t i = 0; i < 18; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "digital_output_" + std::to_string(i),
                                                                     &actual_dig_out_bits_copy_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface("gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
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

  command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "io_async_success", &io_async_success_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));

  for (size_t i = 0; i < 18; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
  }

  return command_interfaces;
}

CallbackReturn URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Path to the urscript code that will be sent to the robot
  std::string script_filename = info_.hardware_parameters["script_filename"];
  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");

  // Connect to UR robot server
  isConnectedToUR = this->ConnectToUR(robot_ip);
  if (!isConnectedToUR){
    RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "Could not connect to Robot!");
  }
  else{
    // Send UR script to robot controller
    this->ProgramFromFile(script_filename);
  }

  async_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncThread, this);

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn URPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  async_thread_shutdown_ = true;
  async_thread_->join();
  async_thread_.reset();

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

void URPositionHardwareInterface::asyncThread()
{
  while (!async_thread_shutdown_) {
    if (initialized_) {
      // RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initialized in async thread");
      // TODO: checkAsyncIO();
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
  }
}

hardware_interface::return_type URPositionHardwareInterface::read()
{
  // Parse status from the UR
  // update the state using the default state updates the UR gives over the specified UR socket
  ur_interface.readFromSocket(urSocket);

  this->JointPos.Position() = vctDoubleVec(ur_interface.cur_joints());
  this->UpdatePositionCartesian(this->CartPos.Position());
  this->JointVel.Velocity() = vctDoubleVec(ur_interface.cur_joint_velocity());
  this->GeoJacobian = ur_interface.getGeoJacobian();
  // this->UpdateVelocityCartesian(this->CartVel.Velocity());

  // pausing state follows runtime state when pausing
  if (runtime_state_ == RuntimeState::PAUSED)
  {
    pausing_state_ = PausingState::PAUSED;
  }
  else if (runtime_state_ == RuntimeState::PLAYING &&
           pausing_state_ == PausingState::PAUSED)
  {
    // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
    speed_scaling_combined_ = 0.0;
    pausing_state_ = PausingState::RAMPUP;
  }

  if (pausing_state_ == PausingState::RAMPUP)
  {
    double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
    speed_scaling_combined_ = std::min(speed_scaling_ramp, speed_scaling_ * target_speed_fraction_);

    if (speed_scaling_ramp > speed_scaling_ * target_speed_fraction_)
    {
      pausing_state_ = PausingState::RUNNING;
    }
  }
  else if (runtime_state_ == RuntimeState::RESUMING)
  {
    // We have to keep speed scaling on ROS side at 0 during RESUMING to prevent controllers from
    // continuing to interpolate
    speed_scaling_combined_ = 0.0;
  }
  else
  {
    // Normal case
    speed_scaling_combined_ = speed_scaling_ * target_speed_fraction_;
  }

  if (first_pass_ && !initialized_)
  {
    initAsyncIO();
    // initialize commands
    ur_position_commands_ = ur_position_commands_old_ = ur_positions_;
    ur_velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    target_speed_fraction_cmd_ = NO_NEW_CMD_;
    resend_robot_program_cmd_ = NO_NEW_CMD_;
    initialized_ = true;
    system_interface_initialized_ = 1.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::write()
{
  if (velocityTimeLimit)
  {
    if (lastVelocityCommand.Norm() > 0.0001 && bigss::time_now_ms() - lastVelocityCommandTime > lastVelocityCommandThresh)
    {
      StopMotion();
      lastVelocityCommand.SetAll(0.0);
    }
  }

  // If there is no interpreting program running on the robot, we do not want to send anything.
  if (position_controller_running_)
  {
    this->SetPositionJoint(ur_position_commands_);
  }
  else if (velocity_controller_running_)
  {
    this->SetVelocityJoint(ur_velocity_commands_);
  }
  else
  {
    // Do something to keep it alive
  }

  return hardware_interface::return_type::OK;
}

void URPositionHardwareInterface::initAsyncIO()
{
  for (size_t i = 0; i < 18; ++i) {
    standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
  }

  for (size_t i = 0; i < 2; ++i) {
    standard_analog_output_cmd_[i] = NO_NEW_CMD_;
  }

  io_async_success_ = 1.0;
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
    ur_position_commands_ = ur_position_commands_old_ = ur_positions_;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    ur_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    ur_position_commands_ = ur_position_commands_old_ = ur_positions_;
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
}  // namespace ur_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
