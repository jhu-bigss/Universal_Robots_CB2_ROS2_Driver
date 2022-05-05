// Copyright (c) 2021 PickNik LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-20
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define UR_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  DIGITAL_OUTPUTS_CMD = 0u,
  ANALOG_OUTPUTS_CMD = 18,
  IO_ASYNC_SUCCESS = 20,
  TARGET_SPEED_FRACTION_CMD = 21,
  TARGET_SPEED_FRACTION_ASYNC_SUCCESS = 22,
  RESEND_ROBOT_PROGRAM_CMD = 23,
  RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS = 24,
};

enum StateInterfaces
{
  DIGITAL_OUTPUTS = 0u,
  DIGITAL_INPUTS = 18,
  ANALOG_OUTPUTS = 36,
  ANALOG_INPUTS = 38,
  INITIALIZED_FLAG = 69,
};

class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  bool setIO(ur_msgs::srv::SetIO::Request::SharedPtr req, ur_msgs::srv::SetIO::Response::SharedPtr resp);

  bool setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                      ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp);

  bool resendRobotProgram(std_srvs::srv::Trigger::Request::SharedPtr req,
                          std_srvs::srv::Trigger::Response::SharedPtr resp);

  void publishIO();

protected:
  void initMsgs();

  bool first_pass_;

  // internal commands
  std::array<double, 18> standard_digital_output_cmd_;
  std::array<double, 18> standard_analog_output_cmd_;
  double target_speed_fraction_cmd_;

  // services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_srv_;
  rclcpp::Service<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr set_speed_slider_srv_;
  rclcpp::Service<ur_msgs::srv::SetIO>::SharedPtr set_io_srv_;

  std::shared_ptr<rclcpp::Publisher<ur_msgs::msg::IOStates>> io_pub_;

  ur_msgs::msg::IOStates io_msg_;

  static constexpr double ASYNC_WAITING = 2.0;
  // TODO(anyone) publishers to add: tcp_pose_pub_
  // TODO(anyone) subscribers to add: script_command_sub_
  // TODO(anyone) service servers to add: resend_robot_program_srv_, deactivate_srv_, tare_sensor_srv_
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__GPIO_CONTROLLER_HPP_
