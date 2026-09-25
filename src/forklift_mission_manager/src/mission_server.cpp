#include "forklift_mission_manager/mission_server.hpp"

namespace forklift_mission_manager
{

Mission MissionServer::goalToMission(const ExecuteMission::Goal & goal)
{
  Mission m;
  m.mission_id = goal.mission_id;
  m.type = missionTypeFromString(goal.mission_type);
  m.pick_area_pose = goal.pick_area_pose;
  m.pick_dock_id = goal.pick_dock_id;
  m.drop_area_pose = goal.drop_area_pose;
  m.drop_dock_id = goal.drop_dock_id;
  m.waypoints = goal.waypoints;
  m.dock_id = goal.dock_id;
  m.station_wait_duration = goal.station_wait_duration;
  return m;
}

MissionServer::MissionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_server", options)
{
  loadParameters();

  mission_database_ = std::make_unique<MissionDatabase>(this->get_logger());
  mission_database_->loadFromFile(mission_database_path_);

  mission_executor_ = std::make_unique<MissionExecutor>(this, execute_task_action_name_);

  action_server_ = rclcpp_action::create_server<ExecuteMission>(
    this, "execute_mission",
    std::bind(&MissionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MissionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&MissionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "mission_server ready. Action: 'execute_mission'. Mission types: PALLET_TRANSPORT, "
    "PATROL, CHARGING, STATION_OPERATION");
}

void MissionServer::loadParameters()
{
  mission_database_path_ = this->declare_parameter("mission_database", std::string(""));
  execute_task_action_name_ =
    this->declare_parameter("execute_task_action_name", std::string("/execute_task"));
}

rclcpp_action::GoalResponse MissionServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteMission::Goal> goal)
{
  if (busy_.load()) {
    RCLCPP_WARN(
      this->get_logger(), "Rejecting mission '%s': another mission is already executing "
      "(no queue at this layer -- caller should retry after completion).",
      goal->mission_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  Mission probe = goalToMission(*goal);

  std::string error_message;
  if (!mission_database_->validateMission(probe, error_message)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting mission: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!expandMissionToTasks(probe, error_message)) {
    RCLCPP_WARN(this->get_logger(), "Rejecting mission: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionServer::handleCancel(
  const std::shared_ptr<GoalHandleExecuteMission>)
{
  RCLCPP_INFO(
    this->get_logger(), "Cancel request accepted; will propagate to the currently "
    "executing task, if any, at the next safe checkpoint.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionServer::handleAccepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
  busy_.store(true);
  std::thread(&MissionServer::execute, this, goal_handle).detach();
}

void MissionServer::execute(const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
  auto goal = goal_handle->get_goal();

  Mission mission = goalToMission(*goal);
  mission.state = MissionState::RECEIVED;
  mission.created_time = this->now();

  mission.state = MissionState::VALIDATING;
  std::string error_message;
  if (!expandMissionToTasks(mission, error_message)) {
    // Shouldn't normally happen (handleGoal already validated), but
    // guards against a race if e.g. missions.yaml changed between
    // accept and here.
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->message = error_message;
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Executing mission '%s' (%s), %zu task(s)",
    mission.mission_id.c_str(), missionTypeToString(mission.type).c_str(), mission.tasks.size());

  mission.state = MissionState::EXECUTING;

  auto feedback_msg = std::make_shared<ExecuteMission::Feedback>();
  feedback_msg->mission_id = mission.mission_id;

  auto feedback_fn =
    [&](
    const std::string & current_task, int task_index, int total_tasks,
    const std::string & state, float progress, const std::string & message)
    {
      feedback_msg->current_task = current_task;
      feedback_msg->current_task_index = task_index;
      feedback_msg->total_tasks = total_tasks;
      feedback_msg->progress = progress;
      feedback_msg->state = state;
      feedback_msg->message = message;
      goal_handle->publish_feedback(feedback_msg);
    };

  auto is_canceling = [goal_handle]() {return goal_handle->is_canceling();};

  MissionOutcome outcome = mission_executor_->run(mission, feedback_fn, is_canceling);

  auto result = std::make_shared<ExecuteMission::Result>();
  result->success = outcome.success;
  result->message = outcome.message;
  result->failed_task = outcome.failed_task;

  if (outcome.success) {
    feedback_msg->state = "COMPLETED";
    feedback_msg->progress = 100.0f;
    feedback_msg->message = "Mission completed";
    goal_handle->publish_feedback(feedback_msg);

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Mission '%s' completed", mission.mission_id.c_str());
  } else if (is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Mission '%s' canceled", mission.mission_id.c_str());
  } else {
    goal_handle->abort(result);
    RCLCPP_ERROR(
      this->get_logger(), "Mission '%s' failed: %s", mission.mission_id.c_str(),
      outcome.message.c_str());
  }

  busy_.store(false);
}

}  // namespace forklift_mission_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(forklift_mission_manager::MissionServer)