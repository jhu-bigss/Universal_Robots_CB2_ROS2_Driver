cmake_minimum_required(VERSION 3.8)
project(bigss_constraint_controller)

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

    include(${CISST_USE_FILE})

    find_package(sawConstraintController REQUIRED)

    include_directories(
        include
        ${sawConstraintController_INCLUDE_DIR}
        )
    link_directories(
        ${sawConstraintController_LIBRARY_DIR}
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

    add_library(bigss_constraint_controller
        ${SOURCE_FILES}
    )
    ament_target_dependencies(
        bigss_constraint_controller
        sawConstraintController
    )

    ament_export_include_directories(include)
    ament_export_libraries(bigss_constraint_controller)

    set_property(TARGET bigss_constraint_controller PROPERTY FOLDER "bigss_constraint_controller")

    install(
        DIRECTORY include/
        DESTINATION include
        PATTERN .svn EXCLUDE
    )

    install(TARGETS bigss_constraint_controller
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

else(cisst_FOUND_AS_REQUIRED)
    message("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif(cisst_FOUND_AS_REQUIRED)
