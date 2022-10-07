cmake_minimum_required(VERSION 3.5)
project(ur_cb2_robot_driver)

# add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(cisst REQUIRED)

include_directories(include)
include (${CISST_USE_FILE})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  std_msgs
  tf2_geometry_msgs
  controller_manager_msgs
  cisst
)

add_library(
  ur_cb2_robot_driver_plugin
  SHARED
  src/hardware_interface.cpp
  src/ur/RobotServer.cpp
  src/ur/RobotInterface.cpp
  src/ur/URDHKinematics.cpp
  src/ur/bigss_debug_util.cpp
)
target_link_libraries(
  ur_cb2_robot_driver_plugin
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
)
target_include_directories(
  ur_cb2_robot_driver_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  ur_cb2_robot_driver_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# prevent pluginlib from using boost
target_compile_definitions(ur_cb2_robot_driver_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface hardware_interface_plugin.xml)


add_executable(ur_ros2_control_node src/ur_ros2_control_node.cpp)
target_include_directories(ur_ros2_control_node PUBLIC ${controller_manager_INCLUDE_DIRS})
target_link_libraries(ur_ros2_control_node ${controller_manager_LIBRARIES})
ament_target_dependencies(ur_ros2_control_node
  controller_interface
  hardware_interface
  rclcpp
)


install(
  TARGETS ur_ros2_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/publisher_forward_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALL
install(
  TARGETS ur_cb2_robot_driver_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY ur_scripts/
  DESTINATION share/${PROJECT_NAME}/ur_scripts
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_cb2_robot_driver_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclpy
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()