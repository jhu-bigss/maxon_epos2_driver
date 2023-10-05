cmake_minimum_required(VERSION 3.8)
project(maxon_epos2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

  # Create a node
  add_executable(maxon_control src/main_maxon_control.cpp)
  cisst_target_link_libraries(maxon_control ${REQUIRED_CISST_LIBRARIES})
  ament_target_dependencies(maxon_control
    rclcpp
    tf2_ros
    diagnostic_msgs
    cisst_msgs
    crtk_msgs
    cisst_ros2_bridge
    cisst_ros2_crtk
    )

  # Install nodes
  install(TARGETS
    maxon_control
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install launch files, config, etc.
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

endif(cisst_FOUND_AS_REQUIRED)