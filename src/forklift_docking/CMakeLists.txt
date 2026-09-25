cmake_minimum_required(VERSION 3.8)
project(forklift_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(forklift_interfaces REQUIRED)

include_directories(include)

# Rear-contact charging maneuver -- shared with forklift_task_manager,
# which drives charging docks directly in the multi-robot bringup.
add_library(rear_charging_maneuver SHARED
  src/rear_charging_maneuver.cpp
)

target_include_directories(rear_charging_maneuver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rear_charging_maneuver
  rclcpp
  geometry_msgs
  nav_msgs
  ros_gz_interfaces
)

add_library(pallet_dock_workflow_component SHARED
  src/dock_database.cpp
  src/pallet_dock_workflow_node.cpp
)

target_include_directories(pallet_dock_workflow_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(pallet_dock_workflow_component yaml-cpp rear_charging_maneuver)

ament_target_dependencies(pallet_dock_workflow_component
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  opennav_docking_msgs
  # yaml-cpp
  forklift_interfaces
)

rclcpp_components_register_node(pallet_dock_workflow_component
  PLUGIN "forklift_docking::PalletDockWorkflowNode"
  EXECUTABLE pallet_dock_workflow_node
)

install(TARGETS rear_charging_maneuver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pallet_dock_workflow_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/StartDocking.srv"

# )

# add_library(forklift_docking_plugins SHARED
#   src/charging_dock.cpp
#   src/station_dock.cpp
#   src/pallet_pick_dock.cpp
#   src/pallet_drop_dock.cpp
#   src/dock_detector.cpp

# )

# ament_target_dependencies(forklift_docking_plugins 
#   rclcpp geometry_msgs std_msgs tf2 tf2_ros pluginlib
# )

# target_include_directories(forklift_docking_plugins PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# pluginlib_export_plugin_description_file(forklift_docking plugin.xml)

# add_executable(dock_manager
#   src/dock_manager.cpp
#   src/dock_controller.cpp
#   src/dock_database.cpp
#   src/state_machine.cpp
# )


# ament_target_dependencies(dock_manager 
#   rclcpp rclcpp_action geometry_msgs std_msgs std_srvs nav2_msgs
#   tf2 tf2_ros pluginlib forklift_interfaces
# )

# target_link_libraries(dock_manager
#   forklift_docking_plugins
#   yaml-cpp
# )

# add_executable(dock_detector_node
#   src/dock_detector_node_main.cpp
#   src/dock_detector.cpp
# )

# ament_target_dependencies(dock_detector_node
#   rclcpp geometry_msgs std_msgs tf2 tf2_ros
# )

# install(TARGETS
#   forklift_docking_plugins
#   dock_manager
#   dock_detector_node
#   ARCHIVE DESTINATION lib 
#   LIBRARY DESTINATION lib 
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include 
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/pallet_dock_node.py
  scripts/fake_battery_node.py
  DESTINATION lib/${PROJECT_NAME}
)



# ament_export_include_directories(include)
# ament_export_libraries(forklift_docking_plugins)
# ament_export_dependencies(rclcpp geometry_msgs std_msgs tf2 tf_ros2 pluginlib forklift_interfaces)


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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs ros_gz_interfaces)

ament_package()
