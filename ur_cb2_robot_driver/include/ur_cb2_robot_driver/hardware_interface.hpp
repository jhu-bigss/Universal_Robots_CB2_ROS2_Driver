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

#ifndef UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <string>
#include <vector>
#include <thread>
#include <bitset>

// ros2_control hw
#include "ur_cb2_robot_driver/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// UR stuff
#include "ur_cb2_robot_driver/ur/RobotServer.h"
#include "ur_cb2_robot_driver/ur/RobotState.h"
#include "ur_cb2_robot_driver/ur/URDHKinematics.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ur_cb2_robot_driver
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum StoppingInterface
{
  NONE,
  STOP_POSITION,
  STOP_VELOCITY
};

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class URPositionHardwareInterface : public hardware_interface::SystemInterface, public RobotServer
{
public:
  // URPositionHardwareInterface();
  // ~URPositionHardwareInterface();

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

protected:
  void extractToolPose();

  std::array<double, 6> ur_position_commands_;
  std::array<double, 6> ur_position_commands_old_;
  std::array<double, 6> ur_velocity_commands_;
  std::array<double, 6> ur_joint_positions_;
  std::array<double, 6> ur_joint_velocities_;
  std::array<double, 6> ur_joint_efforts_;
  URDHKinematics::JacobianMatrix ur_jacobians_;
  std::array<double, 6> ur_tcp_pose_;

  geometry_msgs::msg::TransformStamped tcp_transform_;

  bool packet_read_;

  // robot states
  UniversalRobot::RobotModeData robot_mode_data_;
  UniversalRobot::RobotMode robot_mode_;
  bool controllers_initialized_;

  // asynchronous commands
  double target_speed_fraction_;
  double target_speed_fraction_cmd_;
  double resend_robot_program_cmd_;
  double resend_robot_program_async_success_;
  bool first_pass_;
  bool initialized_;
  double system_interface_initialized_;

  // resources switching aux vars
  std::vector<uint> stop_modes_;
  std::vector<std::string> start_modes_;
  bool position_controller_running_;
  bool velocity_controller_running_;

  // timer to limit sending cmd frequency should be <= 125 Hz
  rclcpp::Time time_last_cmd_send_;
  rclcpp::Time time_now_;
};

}  // namespace ur_cb2_robot_driver

#endif  // UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

