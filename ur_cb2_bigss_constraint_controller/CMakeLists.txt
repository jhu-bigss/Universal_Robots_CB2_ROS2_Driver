cmake_minimum_required(VERSION 3.8)
project(ur_cb2_bigss_constraint_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(REQUIRED_CISST_LIBRARIES
    cisstCommon
    cisstVector
    cisstOSAbstraction
    cisstMultiTask
    cisstParameterTypes
    cisstRobot
    cisstNumerical
)

find_package(cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if(cisst_FOUND_AS_REQUIRED)
    # load cisst configuration
    include(${CISST_USE_FILE})

    find_package(sawConstraintController REQUIRED)

    include_directories(
        ${ur_cb2_bigss_constraint_controller_SOURCE_DIR}
        ${ur_cb2_bigss_constraint_controller_INCLUDE_DIR}
        ${sawConstraintController_INCLUDE_DIR}
        )

    link_directories(
        ${sawConstraintController_LIBRARY_DIR}
    )

    set(ur_cb2_bigss_constraint_controller_HEADER_DIR "include/ur_cb2_bigss_constraint_controller")

    set(HEADER_FILES
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/helper.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsBIGSSVFConfig.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsBIGSSVFController.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsBIGSSVFRCM.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFAbsoluteJointLimits.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFFixOrientation.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFFollowPositionTrajectory.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFFollowSettlingGains.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFJointPenalty.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFJointVelLimits.h
        ${ur_cb2_bigss_constraint_controller_HEADER_DIR}/mtsVFStayOnAxis.h
    )

    set(SOURCE_FILES
        src/helper.cpp
        src/mtsBIGSSVFConfig.cpp
        src/mtsBIGSSVFController.cpp
        src/mtsBIGSSVFRCM.cpp
        src/mtsVFAbsoluteJointLimits.cpp
        src/mtsVFFixOrientation.cpp
        src/mtsVFFollowPositionTrajectory.cpp
        src/mtsVFFollowSettlingGains.cpp
        src/mtsVFJointPenalty.cpp
        src/mtsVFJointVelLimits.cpp
        src/mtsVFStayOnAxis.cpp
    )

    add_library(ur_cb2_bigss_constraint_controller
        ${HEADER_FILES}
        ${SOURCE_FILES}
    )

    target_link_libraries(
        ur_cb2_bigss_constraint_controller
        ${sawConstraintController_LIBRARIES}
    )

    set_property(TARGET ur_cb2_bigss_constraint_controller PROPERTY FOLDER "ur_cb2_bigss_constraint_controller")
    cisst_target_link_libraries(ur_cb2_bigss_constraint_controller ${REQUIRED_CISST_LIBRARIES} ${cisstConstraintController_LIBRARIES})

    # Install targets for headers and library
    install(DIRECTORY
        "${ur_cb2_bigss_constraint_controller_SOURCE_DIR}/include/ur_cb2_bigss_constraint_controller"
        "${ur_cb2_bigss_constraint_controller_BINARY_DIR}/include/ur_cb2_bigss_constraint_controller"
        DESTINATION include
        PATTERN .svn EXCLUDE)

    install(TARGETS ur_cb2_bigss_constraint_controller
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

else(cisst_FOUND_AS_REQUIRED)
    message("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif(cisst_FOUND_AS_REQUIRED)


