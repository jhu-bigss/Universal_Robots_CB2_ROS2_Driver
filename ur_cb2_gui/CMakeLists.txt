cmake_minimum_required(VERSION 3.8)
project(ur_cb2_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Turn on automatic invocation of the MOC, UIC & RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt
     cisstRobot
     cisstNumerical)

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)
  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package(sawConstraintController REQUIRED)

  # fix the sawConstraintController_INCLUDE_DIR and _LIBRARY_DIR by removing "/share"
  string(REPLACE "/share" "" sawConstraintController_INCLUDE_DIR ${sawConstraintController_INCLUDE_DIR})
  string(REPLACE "/share" "" sawConstraintController_LIBRARY_DIR ${sawConstraintController_LIBRARY_DIR})

  find_package(bigss_constraint_controller REQUIRED)

  find_package(bigss_robotic_system REQUIRED)

  include_directories(
      include
      ${sawConstraintController_INCLUDE_DIR}
      )

  link_directories(
    ${sawConstraintController_LIBRARY_DIR}  
    )

  set(SOURCE_FILES
    src/RoboticSystemInputsWidget.cxx
    src/RoboticSystemConstraintsWidget.cxx
    src/RoboticSystemControlWidget.cxx
    src/RoboticSystemStatusWidget.cxx
    # src/RoboticSystemLoggingWidget.cxx
    src/RoboticSystemWidget.cxx
  )

  set(CMAKE_AUTOUIC_SEARCH_PATHS ui)

  add_library(ur_cb2_gui
    ${SOURCE_FILES} 
    ${UI_FILES}
  )
  ament_target_dependencies(ur_cb2_gui
    cisst
    Qt5Widgets
    sawConstraintController
    bigss_constraint_controller
    bigss_robotic_system
  )

  ament_export_include_directories(include)
  ament_export_libraries(ur_cb2_gui)


  install(
      DIRECTORY include/
      DESTINATION include
  )

  install (TARGETS ur_cb2_gui COMPONENT ur_cb2_gui
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