cmake_minimum_required(VERSION 3.8)
project(ur_cb2_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

  if (cisst_FOUND_AS_REQUIRED)
    # load cisst configuration
    include (${CISST_USE_FILE})

    cisst_find_saw_component (ur_cb2_bigss_constraint_controller REQUIRED)
    find_package(ur_cb2_bigss_constraint_controller REQUIRED)
    include_directories(${ur_cb2_bigss_constraint_controller_INCLUDE_DIR})
    link_directories(${ur_cb2_bigss_constraint_controller_LIBRARY_DIR})

    include_directories(
        include
        ${ur_cb2_bigss_constraint_controller_INCLUDE_DIR}
        )
    link_directories(
        ${ur_cb2_bigss_constraint_controller_LIBRARY_DIR}
    )

    set(SOURCE_FILES
      RoboticSystemInputsWidget.cxx
      RoboticSystemConstraintsWidget.cxx
      RoboticSystemControlWidget.cxx
      RoboticSystemStatusWidget.cxx
      # RoboticSystemLoggingWidget.cxx
      RoboticSystemWidget.cxx
    )

    add_library(ur_cb2_gui
      ${SOURCE_FILES} 
    )
    ament_target_dependencies(ur_cb2_gui
      ${REQUIRED_CISST_LIBRARIES}
    )

    set (ur_cb2_gui_LIBRARIES ur_cb2_gui PARENT_SCOPE)

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