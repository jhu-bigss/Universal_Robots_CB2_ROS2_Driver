#ifndef __ITIA_UR_HARDWARE_INTERFACE__
#define __ITIA_UR_HARDWARE_INTERFACE__


#include <memory>
#include <string>
#include <vector>
#include <limits>

// #include "rclcpp/rclcpp.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// // UR stuff
// #include "ur_client_library/ur_driver.h"
// #include "ur_robot_driver/dashboard_client_ros.hpp"
// #include "ur_dashboard_msgs/msg/robot_mode.hpp"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "controller_interface/controller_interface_base.hpp"
#include <ur_robot_driver_cb2/ur_driver.h>


// using vector6d_t = std::array<double, 6>;
using vector6d_t = std::vector<double>;


namespace cb2_hw
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

  class UrRobotHW: public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UrRobotHW);
    UrRobotHW();
    virtual ~UrRobotHW()
    {
    };

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;
    
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

    std::vector<std::string> m_joint_names;//TODO
    
    std::vector<double> m_tcp_force;
    std::vector<double> m_curr2torque;
    
    // uint32_t runtime_state_;
    PausingState pausing_state_;
    double pausing_ramp_up_increment_;
    bool controllers_initialized_;
    bool first_pass_;
    bool initialized_;
    std::ofstream out;


    
    // UrDriver Variables
    std::unique_ptr<cb2_hw::UrDriver> m_driver;
    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;
    int m_max_velocity;
    
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;

    // unsigned int m_nAx;
    // unsigned int m_missing_messages;
    // unsigned int m_max_missing_messages;
    // bool m_topic_received;
    
    double m_max_vel_change;
    
    // ros::Publisher m_target_pub;
    // ros::Publisher m_force_pub;
    bool m_velocity_mode;
    bool m_position_mode;
    bool m_posvel_mode;
    double m_max_accepted_deviation;
    int m_servogain;
    double m_lookahead;
    int m_acceleration_coeff;
    
  };
}

#endif
