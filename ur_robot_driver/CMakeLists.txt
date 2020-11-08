cmake_minimum_required(VERSION 3.5)
project(ur_robot_driver)

add_definitions( -DROS_BUILD )

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

#find_package(catkin REQUIRED
#  COMPONENTS
#    actionlib
#    control_msgs
#    controller_manager
#    geometry_msgs
#    hardware_interface
#    industrial_robot_status_interface
#    pluginlib
#    roscpp
#    sensor_msgs
#    std_srvs
#    tf
#    tf2_geometry_msgs
#    tf2_msgs
#    trajectory_msgs
#    ur_controllers
#    ur_dashboard_msgs
#    ur_msgs
#)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(ur_robot_driver_plugin
#  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
)
target_link_libraries(ur_robot_driver_plugin ur_client_library::urcl)
ament_target_dependencies(ur_robot_driver_plugin
  hardware_interface
  pluginlib
  std_srvs
  tf2
  tf2_geometry_msgs
  ur_msgs
  ur_dashboard_msgs
)
# add_dependencies(ur_robot_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
# add_executable(ur_robot_driver_node
#   src/dashboard_client_ros.cpp
#   src/hardware_interface.cpp
#   src/hardware_interface_node.cpp
# )
# target_link_libraries(ur_robot_driver_node ${catkin_LIBRARIES} ur_client_library::urcl)
# add_dependencies(ur_robot_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
# add_executable(dashboard_client
#   src/dashboard_client_ros.cpp
#   src/dashboard_client_node.cpp
# )
# target_link_libraries(dashboard_client ${catkin_LIBRARIES} ur_client_library::urcl)
# add_dependencies(dashboard_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
# add_executable(robot_state_helper
#   src/robot_state_helper.cpp
#   src/robot_state_helper_node.cpp
# )
# target_link_libraries(robot_state_helper ${catkin_LIBRARIES} ur_client_library::urcl)
# add_dependencies(robot_state_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_load_params src/test_load_params.cpp)
ament_target_dependencies(test_load_params rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ur_robot_driver_plugin
  # ur_robot_driver_node
  # robot_state_helper
  # dashboard_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME} ur_robot_driver_plugin)
#
# install(PROGRAMS scripts/tool_communication
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
# install(DIRECTORY config launch resources
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
#
# install(FILES hardware_interface_plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

ament_package()
