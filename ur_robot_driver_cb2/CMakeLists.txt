cmake_minimum_required(VERSION 3.8)
project(ur_robot_driver_cb2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
  controller_manager_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

add_library(
  ur_robot_driver_cb2_plugin
  SHARED
  src/ur_hardware_interface.cpp
  src/ur_driver.cpp
  src/robot_state.cpp
  src/robot_state_RT.cpp
  src/ur_communication.cpp
  src/ur_realtime_communication.cpp
)
target_include_directories(
  ur_robot_driver_cb2_plugin
  PUBLIC
  include
)
ament_target_dependencies(
  ur_robot_driver_cb2_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# prevent pluginlib from using boost
target_compile_definitions(ur_robot_driver_cb2_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface hardware_interface_plugin.xml)

install(
  TARGETS
  ur_robot_driver_cb2_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_ur_robot_driver_cb2 test/test_ur_robot_driver_cb2.cpp)
  target_include_directories(test_ur_robot_driver_cb2 PRIVATE include)
  ament_target_dependencies(
    test_ur_robot_driver_cb2
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_robot_driver_cb2_plugin
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
