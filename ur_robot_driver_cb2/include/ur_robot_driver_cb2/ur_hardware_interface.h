#ifndef __BIGSS_UR_HARDWARE_INTERFACE__
#define __BIGSS_UR_HARDWARE_INTERFACE__


#include <memory>
#include <string>
#include <vector>
#include <limits>

// #include "rclcpp/rclcpp.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"


// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "controller_interface/controller_interface_base.hpp"
#include <ur_robot_driver_cb2/ur_driver.h>


using vector6d_t = std::array<double, 6>;
using vector3d_t = std::array<double, 3>;

namespace ur_robot_driver_cb2
{
  
  enum class PausingState
  {
    PAUSED,
    RUNNING,
    RAMPUP
  };

  enum StoppingInterface
  {
    NONE,
    STOP_POSITION,
    STOP_VELOCITY
  };

  class URPositionHardwareInterface: public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(URPositionHardwareInterface);
    URPositionHardwareInterface();
    virtual ~URPositionHardwareInterface()
    {
    };

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;
    
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

    
  protected:
    
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr torque_pub_;

    geometry_msgs::msg::Wrench wrench_message_ = geometry_msgs::msg::Wrench();
    
    vector6d_t urcl_position_commands_;
    vector6d_t urcl_position_commands_old_;
    vector6d_t urcl_velocity_commands_;
    vector6d_t urcl_joint_positions_;
    vector6d_t urcl_joint_velocities_;
    vector6d_t urcl_joint_efforts_;
    vector6d_t urcl_ft_sensor_measurements_;
    vector6d_t urcl_tcp_pose_;

    double speed_scaling_combined_;

    std::vector<std::string> joint_names_;
    
    std::vector<double> tcp_force_;
    std::vector<double> curr2torque_;
    
    // uint32_t runtime_state_;
    PausingState pausing_state_;
    double pausing_ramp_up_increment_;
    bool controllers_initialized_;
    bool first_pass_;
    bool initialized_;
    std::ofstream out;

    // UrDriver Variables
    std::unique_ptr<ur_robot_driver_cb2::UrDriver> ur_driver_;
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    int max_velocity_;
    
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;

    // unsigned int m_nAx;
    // unsigned int m_missing_messages;
    // unsigned int m_max_missing_messages;
    // bool m_topic_received;
    
    double max_vel_change_;
    
    // ros::Publisher m_target_pub;
    // ros::Publisher m_force_pub;
    bool velocity_mode_;
    bool position_mode_;
    bool posvel_mode_;
    double max_accepted_deviation_;
    int servogain_;
    double lookahead_;
    int acceleration_coeff_;

    double max_payload_;
    
    // GPIO dummy variables
    std::array<double, 18> actual_dig_out_bits_;
    std::array<double, 18> actual_dig_in_bits_;
    std::array<double, 11> safety_status_bits_;
    std::array<double, 4> analog_io_types_;
    std::array<double, 4> robot_status_bits_;
    std::array<double, 2> tool_analog_input_types_;
    std::array<double, 2> tool_analog_input_;
    std::array<double, 2> standard_analog_input_;
    std::array<double, 2> standard_analog_output_;
    double tool_output_voltage_;
    double robot_mode_;
    double safety_mode_;
    double tool_mode_;
    double tool_output_current_;
    double tool_temperature_;
    double system_interface_initialized_;
    double robot_program_running_;

    double io_async_success_;
    double target_speed_fraction_cmd_;
    double scaling_async_success_;
    double resend_robot_program_cmd_;
    double resend_robot_program_async_success_;
    double hand_back_control_cmd_;
    double hand_back_control_async_success_;
    double payload_mass_;
    vector3d_t payload_center_of_gravity_;
    double payload_async_success_;
    std::array<double, 18> standard_dig_out_bits_cmd_;
    std::array<double, 2> standard_analog_output_cmd_;
    double tool_voltage_cmd_;
    double zero_ftsensor_cmd_;
    double zero_ftsensor_async_success_;

  };
}

#endif
