cmake_minimum_required(VERSION 3.8)
project(forklift_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoMarkers.msg"
  "srv/StartDocking.srv"
  "msg/Task.msg"
  "msg/TaskStatus.msg"
  "msg/TaskResult.msg"
  "msg/NavigationRequest.msg"
  "msg/NavigationResult.msg"
  "msg/DockingResult.msg"
  "msg/DockingRequest.msg"
  "msg/LiftRequest.msg"
  "msg/LiftResult.msg"

  # --- forklift_controller ---
  "msg/LiftMotorCommand.msg"
  "msg/CurrentHeight.msg"
  "msg/LiftLimitStatus.msg"
  "msg/LiftEncoderHeight.msg"
  "msg/LiftDriverStatus.msg"

  # --- forklift_mission_manager ---
  "msg/Mission.msg"
  "msg/MissionStatus.msg"
  "msg/MissionStep.msg"

  # --- forklift_safety ---
  "msg/UltrasonicArray.msg"
  "msg/BumperState.msg"
  "msg/SafetyStatus.msg"
  "msg/ProtectiveFieldStatus.msg"
  "msg/ObstacleState.msg"
  "msg/SafetyIOInput.msg"
  "msg/SafetyIOStatus.msg"
  "msg/SensorStatus.msg"
  # --- forklift_diagnostics ---
  "msg/DeviceStatus.msg"
  "msg/HardwareStatus.msg"
  "msg/SensorDiagnostics.msg"
  "msg/SystemStatus.msg"
  "msg/CommunicationStatus.msg"
  "msg/MaintenanceItem.msg"
  "msg/MaintenanceStatus.msg"
  "msg/DiagnosticsAlert.msg"
  "msg/DiagnosticsStatus.msg"
  "msg/RobotHealth.msg"

  # --- forklift_web_bridge ---
  "msg/RobotState.msg"
  "msg/WebBridgeStatus.msg"
  "msg/NavigationCommand.msg"
  "msg/NavigationStatus.msg"
  "msg/MissionCommand.msg"
  
  "action/DockRobot.action"
  "action/UndockRobot.action"
  "action/SetLiftHeight.action"
  "action/PalletDockWorkflow.action"
  "srv/SetLiftHeight.srv"

  # --- forklift_task_manager -- 
  "action/ExecuteTask.action"
  "msg/TaskManagerStatus.msg"
  "msg/TaskStepStatus.msg"
  "srv/GetTaskStatus.srv"
  "srv/ClearQueue.srv"

  # --- forklift_mission_manager ---
  "action/ExecuteMission.action"

  # --- forklift_traffic_manager ---
  "srv/ReserveEdge.srv"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
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
