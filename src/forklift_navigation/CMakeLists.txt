cmake_minimum_required(VERSION 3.8)
project(forklift_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(topic_tools REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)


install(DIRECTORY launch map rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install( DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

add_library(forklift_concatenate_paths_bt_node SHARED
  src/concatenate_paths_bt_node.cpp
)
ament_target_dependencies(forklift_concatenate_paths_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
)
target_compile_definitions(forklift_concatenate_paths_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS forklift_concatenate_paths_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(forklift_publish_path_bt_node SHARED
  src/publish_path_bt_node.cpp
)
ament_target_dependencies(forklift_publish_path_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
  rclcpp
)
target_compile_definitions(forklift_publish_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS forklift_publish_path_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/remapper.py
  DESTINATION lib/${PROJECT_NAME}
)

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
