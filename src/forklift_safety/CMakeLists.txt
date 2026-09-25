cmake_minimum_required(VERSION 3.8)
project(forklift_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(forklift_interfaces REQUIRED)


include_directories(include)
set(DEPS rclcpp std_msgs std_srvs sensor_msgs nav_msgs geometry_msgs forklift_interfaces)
set(NODES 
  safety_manager_node
  emergency_stop_node
  obstacle_monitor_node
  protective_field_node
  speed_zone_node
  sensor_monitor_node
  safety_io_node
)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} ${DEPS})
endforeach()

install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})


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
