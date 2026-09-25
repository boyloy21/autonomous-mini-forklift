cmake_minimum_required(VERSION 3.8)
project(forklift_traffic_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_route REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(forklift_interfaces REQUIRED)

include_directories(include)

# Fleet-wide coordinator node
add_executable(traffic_manager_node src/traffic_manager_node.cpp)
ament_target_dependencies(traffic_manager_node
  rclcpp
  nav2_msgs
  forklift_interfaces
)

# route_server RouteOperation plugin, loaded per-robot
add_library(traffic_reservation SHARED src/traffic_reservation.cpp)
ament_target_dependencies(traffic_reservation
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_route
  nav2_costmap_2d
  geometry_msgs
  forklift_interfaces
)
# nav2_route's installed headers live under .../include/nav2_route/nav2_route/,
# and its own internal headers include each other as "nav2_route/foo.hpp"
# (not doubled) -- so the inner directory needs to be on the include path
# too, matching how nav2_route's own plugins are built. nav2_route_INCLUDE_DIRS
# is a semicolon-separated list -- take its first entry (the package's own
# include root, per nav2_route's ament_cmake_export_include_directories
# extras) rather than string-appending to the whole list.
list(GET nav2_route_INCLUDE_DIRS 0 _nav2_route_include_dir)
target_include_directories(traffic_reservation PRIVATE ${_nav2_route_include_dir}/nav2_route)

pluginlib_export_plugin_description_file(nav2_route route_operations_plugins.xml)

install(TARGETS traffic_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS traffic_reservation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES route_operations_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
