cmake_minimum_required (VERSION 3.10)
project (ur_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (REQUIRED_CISST_LIBRARIES
     cisstParameterTypes
     cisstCommon
     cisstMultiTask
     )

find_package (cisst COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  add_library (URInterface
               INTERFACE
               )
  ament_target_dependencies(URInterface INTERFACE
    cisst
  )

  ament_export_include_directories(include)
  ament_export_libraries(URInterface)

  set_property (TARGET URInterface PROPERTY FOLDER "URInterface")
  set_target_properties(URInterface PROPERTIES LINKER_LANGUAGE CXX)

  # Install targets for headers and library
  install (DIRECTORY include/
           DESTINATION include
  )

  install (TARGETS URInterface
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

ament_export_dependencies(cisst bigss_robotic_system)
ament_package()
           
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
