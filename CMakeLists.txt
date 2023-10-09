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

if (cisst_FOUND_AS_REQUIRED)

  include(${CISST_USE_FILE})

  # Include directories
  include_directories(
    include
  )
  # Source files
  set(maxon_epos2_driver_SOURCES
    src/bigss_debug_util.cpp
    src/main_maxon_control.cpp
    src/maxonControlWidget.cpp
    src/maxonInterface.cpp
    src/maxonMotor.cpp
    src/maxonMotorInterface.cpp
    src/maxonStatusWidget.cpp
    src/maxonWidget.cpp
  )

  # Create a node
  add_executable(maxon_epos2_driver ${maxon_epos2_driver_SOURCES})
  cisst_target_link_libraries(maxon_epos2_driver ${REQUIRED_CISST_LIBRARIES})
  ament_target_dependencies(maxon_epos2_driver
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

  # Install nodes
  install(TARGETS
    maxon_epos2_driver
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install launch files, config, etc.
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

endif(cisst_FOUND_AS_REQUIRED)