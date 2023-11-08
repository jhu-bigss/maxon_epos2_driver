cmake_minimum_required(VERSION 3.8)
project(maxon_epos2_driver)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cisst_msgs REQUIRED)
find_package(crtk_msgs REQUIRED)
find_package(cisst_ros2_bridge REQUIRED)
find_package(cisst_ros2_crtk REQUIRED)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt
     )

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

# turn on Qt code autogeneration
# need to add headers to the add_executable and add_library calls
set (CMAKE_AUTOMOC ON)

if (cisst_FOUND_AS_REQUIRED)

  include(${CISST_USE_FILE})

  # Include directories
  include_directories(
    include
  )

  # Create a library
  add_library(maxon_interface
    src/maxonInterface.cpp
    src/maxonMotor.cpp
    src/maxonMotorInterface.cpp
    include/maxon_epos2_driver/maxonInterface.h
    include/maxon_epos2_driver/maxonMotor.h
    include/maxon_epos2_driver/maxonMotorInterface.h
    )
  target_link_libraries(maxon_interface
    -lEposCmd
    )
  cisst_target_link_libraries(maxon_interface ${REQUIRED_CISST_LIBRARIES})
  ament_target_dependencies(maxon_interface
    rclcpp
    tf2_ros
    std_srvs
    std_msgs
    diagnostic_msgs
    cisst_msgs
    crtk_msgs
    cisst_ros2_bridge
    cisst_ros2_crtk
    )

  # Create a node
  add_executable(maxon_epos2_driver
    src/main_maxon_control.cpp
    )
  target_link_libraries(maxon_epos2_driver
    maxon_interface
    )

  add_executable(maxon_epos2_driver_gui
    src/main_maxon_ui_control.cpp
    src/maxonWidget.cpp
    src/maxonControlWidget.cpp
    src/maxonStatusWidget.cpp
    include/maxon_epos2_driver/maxonWidget.h
    include/maxon_epos2_driver/maxonControlWidget.h
    include/maxon_epos2_driver/maxonStatusWidget.h
    )
  target_link_libraries(maxon_epos2_driver_gui
    maxon_interface
    )

  ## INSTALL ##
  install(TARGETS
    maxon_epos2_driver
    maxon_epos2_driver_gui
    DESTINATION lib/${PROJECT_NAME}
    )

  # Install launch files, config, etc.
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
    )

  install (
    TARGETS maxon_interface
    EXPORT maxon_interface
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    )

  ament_package()

endif(cisst_FOUND_AS_REQUIRED)