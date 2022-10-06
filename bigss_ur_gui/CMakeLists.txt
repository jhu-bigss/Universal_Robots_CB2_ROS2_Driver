cmake_minimum_required(VERSION 3.8)
project(bigss_ur_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Turn on automatic invocation of the MOC, UIC & RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set (CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
# find_package(Qt5Widgets REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

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

  set(HEADER_FILES
    include/${PROJECT_NAME}/RoboticSystemInputsWidget.h
    include/${PROJECT_NAME}/RoboticSystemConstraintsWidget.h
    include/${PROJECT_NAME}/RoboticSystemControlWidget.h
    include/${PROJECT_NAME}/RoboticSystemStatusWidget.h
    # include/${PROJECT_NAME}/RoboticSystemLoggingWidget.h
    include/${PROJECT_NAME}/RoboticSystemWidget.h
  )

  set(CMAKE_AUTOUIC_SEARCH_PATHS ui)

  add_library(bigss_ur_gui
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${UI_FILES}
  )
  ament_target_dependencies(bigss_ur_gui
    cisst
    Qt5Widgets
    sawConstraintController
    bigss_constraint_controller
    bigss_robotic_system
  )

  ament_export_include_directories(include)
  ament_export_libraries(bigss_ur_gui)

  install(
      DIRECTORY include/
      DESTINATION include
  )

  install (TARGETS bigss_ur_gui COMPONENT bigss_ur_gui
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

endif (cisst_FOUND_AS_REQUIRED)