cmake_minimum_required(VERSION 3.8)
project(forklift_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(forklift_interfaces REQUIRED)


set(DEPS rclcpp rclcpp_action std_msgs sensor_msgs action_msgs forklift_interfaces)
set(NODES lift_controller_node lift_controller_node_multirobot lift_driver_node lift_feedback_node lift_limit_node lift_safety_node lift_cmd_bridge_node)
 
foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} ${DEPS})
endforeach()
 
install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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
