cmake_minimum_required(VERSION 3.8)
project(bigss_spine_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)


find_package(Qt5Widgets REQUIRED
  )

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     )

find_package(cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)
  # load cisst configuration
  include (${CISST_USE_FILE})
endif (cisst_FOUND_AS_REQUIRED)

find_package(cisst_msgs REQUIRED)
find_package(cisst_ros2_bridge REQUIRED)
find_package(cisst_ros2_crtk REQUIRED)

find_package(sawConstraintController REQUIRED)
# fix the sawConstraintController_INCLUDE_DIR and _LIBRARY_DIR by removing "/share"
string(REPLACE "/share" "" sawConstraintController_INCLUDE_DIR ${sawConstraintController_INCLUDE_DIR})
string(REPLACE "/share" "" sawConstraintController_LIBRARY_DIR ${sawConstraintController_LIBRARY_DIR})

find_package(bigss_constraint_controller REQUIRED)
find_package(bigss_robotic_system REQUIRED)

find_package(ur_cb2_gui REQUIRED)

include_directories(${sawConstraintController_INCLUDE_DIR})

link_directories(${sawConstraintController_LIBRARY_DIR})

add_executable(main_ur_controller_with_snake src/node_ur_with_snake.cpp)
ament_target_dependencies(main_ur_controller_with_snake
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
  diagnostic_msgs
  cisst
  cisst_msgs
  cisst_ros2_bridge
  cisst_ros2_crtk
  Qt5Widgets
  sawConstraintController
  bigss_constraint_controller
  bigss_robotic_system
  ur_cb2_gui
)

install(TARGETS
  main_ur_controller_with_snake
  DESTINATION lib/${PROJECT_NAME}
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
endif()

ament_package()
