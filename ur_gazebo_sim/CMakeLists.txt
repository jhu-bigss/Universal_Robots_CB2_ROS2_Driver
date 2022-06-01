cmake_minimum_required(VERSION 3.8)
project(ur_gazebo_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ex_follow_target src/ex_follow_target.cpp)
ament_target_dependencies(ex_follow_target
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)
install(TARGETS
    ex_follow_target
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
