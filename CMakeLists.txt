add_library(
  ur_robot_driver
  SHARED
  src/hardware_interface.cpp
)
target_include_directories(
  ur_robot_driver
  PUBLIC
  include
)
ament_target_dependencies(
  ur_robot_driver
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(ur_robot_driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ur_robot_driver.xml)

install(
  TARGETS
  ur_robot_driver
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


  ament_add_gmock(test_hardware_interface test/test_hardware_interface.cpp)
  target_include_directories(test_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )

ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_robot_driver
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
