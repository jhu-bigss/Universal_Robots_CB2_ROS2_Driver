cmake_minimum_required(VERSION 3.8)
project(bigss_robotic_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     )

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)
  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package(sawConstraintController REQUIRED)
  find_package(bigss_constraint_controller REQUIRED)

  # fix the sawConstraintController_INCLUDE_DIR and _LIBRARY_DIR by removing "/share"
  string(REPLACE "/share" "" sawConstraintController_INCLUDE_DIR ${sawConstraintController_INCLUDE_DIR})
  string(REPLACE "/share" "" sawConstraintController_LIBRARY_DIR ${sawConstraintController_LIBRARY_DIR})

  include_directories(
      include
      ${sawConstraintController_INCLUDE_DIR}
      )

  link_directories(
    ${sawConstraintController_LIBRARY_DIR}  
    )

  set(SOURCE_FILES
    src/KinematicsObject.cpp
    src/mtsSARobSysTask.cpp
    src/RobotInterfaceObject.cpp
    src/SensorInterface.cpp
    src/SeriallyAttachedRoboticSystem.cpp
  )

  add_library(bigss_robotic_system
    ${SOURCE_FILES} 
  )
  ament_target_dependencies(bigss_robotic_system
    cisst
    sawConstraintController
    bigss_constraint_controller
  )

  ament_export_include_directories(include)
  ament_export_libraries(bigss_robotic_system)

  set_property(TARGET bigss_robotic_system PROPERTY FOLDER "bigss_robotic_system")

  install(
      DIRECTORY include/
      DESTINATION include
  )

  install(TARGETS bigss_robotic_system
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

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

endif (cisst_FOUND_AS_REQUIRED)