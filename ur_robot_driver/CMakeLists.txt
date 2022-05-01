cmake_minimum_required(VERSION 3.5)
project(ur_robot_driver)

add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  std_msgs
  controller_manager_msgs
)

add_library(
  ur_robot_driver_plugin
  SHARED
  src/hardware_interface.cpp
)
target_include_directories(
  ur_robot_driver_plugin
  PUBLIC
  include
)
ament_target_dependencies(
  ur_robot_driver_plugin
  hardware_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# prevent pluginlib from using boost
target_compile_definitions(ur_robot_driver_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface hardware_interface_plugin.xml)

install(
  TARGETS
  ur_robot_driver_plugin
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


  # ament_add_gmock(test_hardware_interface test/test_hardware_interface.cpp)
  # target_include_directories(test_hardware_interface PRIVATE include)
  # ament_target_dependencies(
  #   test_hardware_interface
  #   hardware_interface
  #   pluginlib
  #   ros2_control_test_assets
  # )

ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_robot_driver_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
