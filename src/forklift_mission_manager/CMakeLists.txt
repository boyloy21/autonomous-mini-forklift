cmake_minimum_required(VERSION 3.8)
project(forklift_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(forklift_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
 
add_library(${PROJECT_NAME}_component SHARED
  src/mission_state_machine.cpp
  src/mission.cpp
  src/mission_database.cpp
  src/mission_executor.cpp
  src/mission_server.cpp
)
 
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
 
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  forklift_interfaces
)
 
target_link_libraries(${PROJECT_NAME}_component yaml-cpp)
 
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "forklift_mission_manager::MissionServer"
  EXECUTABLE mission_server
)
 
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
