cmake_minimum_required(VERSION 3.8)
project(forklift_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(forklift_interfaces REQUIRED)
find_package(forklift_docking REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/dock_database.cpp
  src/task.cpp
  src/task_queue.cpp
  src/task_executor.cpp
  src/task_manager_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rcl_interfaces
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav2_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  opennav_docking_msgs
  forklift_interfaces
  forklift_docking
)

target_link_libraries(${PROJECT_NAME}_component yaml-cpp)
 
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "forklift_task_manager::TaskManagerNode"
  EXECUTABLE task_manager_node
)

# include_directories(include)
 
# set(DEPS
#   rclcpp
#   rclcpp_action
#   rclcpp_components
#   std_msgs
#   std_srvs
#   geometry_msgs
#   nav2_msgs
#   forklift_interfaces
# )

# set(NODES
#   task_scheduler_node
#   task_queue_node
#   task_executor_node
#   navigation_client_node
#   docking_client_node
#   lift_client_node
#   recovery_node
#   safety_node
#   task_monitor_node
# )

# foreach(node ${NODES})
#   add_executable(${node} src/${node}.cpp)
#   ament_target_dependencies(${node} ${DEPS})
# endforeach()

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
 
install(DIRECTORY include/
  DESTINATION include
)
 
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
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
