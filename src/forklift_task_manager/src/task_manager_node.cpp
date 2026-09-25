#include "forklift_task_manager/task_manager_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace forklift_task_manager
{

GoalParams TaskManagerNode::goalToParams(const ExecuteTask::Goal & goal)
{
  GoalParams p;
  p.goal_pose = goal.goal_pose;
  p.waypoints = goal.waypoints;
  p.dock_id = goal.dock_id;
  p.target_height = goal.target_height;
  p.station_wait_duration = goal.station_wait_duration;
  return p;
}

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_manager_node", options)
{
  loadParameters();

  executor_ = std::make_unique<TaskExecutor>(
    this,
    this->declare_parameter("navigate_action_name", std::string("navigate_to_pose")),
    this->declare_parameter("dock_action_name", std::string("dock_robot")),
    this->declare_parameter("undock_action_name", std::string("undock_robot")),
    this->declare_parameter("lift_action_name", std::string("set_height")),
    this->declare_parameter("aruco_node_name", std::string("aruco_node")),
    dock_database_path_,
    this->declare_parameter("battery_topic", std::string("battery_state")),
    this->declare_parameter("charging_current_threshold", 0.5),
    this->declare_parameter("charging_monitor_timeout", 300.0));

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this, "execute_task",
    std::bind(&TaskManagerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskManagerNode::handleCancel, this, std::placeholders::_1),
    std::bind(&TaskManagerNode::handleAccepted, this, std::placeholders::_1));

  get_status_srv_ = this->create_service<GetTaskStatus>(
    "get_task_status",
    std::bind(
      &TaskManagerNode::handleGetTaskStatus, this, std::placeholders::_1, std::placeholders::_2));

  clear_queue_srv_ = this->create_service<ClearQueue>(
    "clear_task_queue",
    std::bind(
      &TaskManagerNode::handleClearQueue, this, std::placeholders::_1, std::placeholders::_2));

  status_pub_ = this->create_publisher<forklift_interfaces::msg::TaskManagerStatus>(
    "task_manager/status", 10);

  // Step-level stream for observers (forklift_web_bridge's mission_status,
  // dashboards). Unlike ExecuteTask feedback, which only the goal sender
  // receives, this is a plain topic. Transient local so a client that
  // connects mid-task immediately sees the current step.
  step_status_pub_ = this->create_publisher<forklift_interfaces::msg::TaskStepStatus>(
    "task_status", rclcpp::QoS(10).transient_local());

  if (use_docking_logs_) {
    const std::string ns = std::string(this->get_namespace());
    // Logger names carry the namespace with a dot, e.g.
    // "forklift1.docking_server"; other robots' lines are ignored.
    docking_logger_name_ = (ns == "/" ? "" : ns.substr(1) + ".") + "docking_server";
    docking_log_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100),
      std::bind(&TaskManagerNode::onLogMessage, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "Reporting docking phases from '%s' log output on task_status",
      docking_logger_name_.c_str());
  }

  // Docking sub-steps (DETECT / APPROACH / WAIT_FOR_CHARGE) are only
  // visible to the DockRobot action client inside the executor.
  executor_->setSubStepCallback(
    [this](const std::string & step, const std::string & state, const std::string & message) {
      {
        std::lock_guard<std::mutex> lock(step_context_mutex_);
        if (step == "DETECT" && state == "SUCCESS") {step_context_.detected = true;}
      }
      this->publishStepStatus(step, state, message);
    });

  status_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / status_publish_rate_hz_)),
    std::bind(&TaskManagerNode::publishStatus, this));

  worker_thread_ = std::thread(&TaskManagerNode::workerLoop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "task_manager_node ready. Action: 'execute_task'. Task types: SINGLE_NAVIGATION, "
    "PATROL_POINT, PICK_PALLET_DOCK, DROP_PALLET_DOCK, CHARGING_DOCK, STATION_DOCK");
}

TaskManagerNode::~TaskManagerNode()
{
  running_.store(false);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void TaskManagerNode::loadParameters()
{
  dock_database_path_ = this->declare_parameter("dock_database", std::string(""));
  staging_x_offset_ = this->declare_parameter("staging_x_offset", -0.7);
  staging_yaw_offset_ = this->declare_parameter("staging_yaw_offset", 0.0);
  status_publish_rate_hz_ = this->declare_parameter("status_publish_rate_hz", 2.0);
  use_docking_logs_ = this->declare_parameter("use_docking_logs", true);
}

// ==================== ExecuteTask action server ====================

rclcpp_action::GoalResponse TaskManagerNode::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
{
  const TaskType type = taskTypeFromString(goal->task_type);
  if (type == TaskType::UNKNOWN) {
    RCLCPP_WARN(
      this->get_logger(), "Rejecting goal: unknown task_type '%s'. Valid: SINGLE_NAVIGATION, "
      "PATROL_POINT, PICK_PALLET_DOCK, DROP_PALLET_DOCK, CHARGING_DOCK, STATION_DOCK",
      goal->task_type.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  Task probe;
  probe.type = type;
  probe.goal_params = goalToParams(*goal);

  std::string error_message;
  if (!expandTaskToSteps(
      probe, dock_database_path_, staging_x_offset_, staging_yaw_offset_, error_message))
  {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskManagerNode::handleCancel(
  const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
{
  std::string queued_task_id;
  {
    std::lock_guard<std::mutex> lock(pending_handles_mutex_);
    for (auto it = pending_goal_handles_.begin(); it != pending_goal_handles_.end(); ++it) {
      if (it->second == goal_handle) {
        queued_task_id = it->first;
        pending_goal_handles_.erase(it);
        break;
      }
    }
  }

  if (!queued_task_id.empty()) {
    queue_.remove(queued_task_id);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = false;
    result->error_code = "CANCELED";
    result->message = "Canceled while queued (never started executing)";
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Canceled queued task '%s'", queued_task_id.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Cancel request accepted for currently-executing task; "
      "worker thread will stop at the next safe checkpoint.");
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskManagerNode::handleAccepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
{
  auto goal = goal_handle->get_goal();

  Task task;
  task.task_id = goal->task_id.empty() ?
    (goal->task_type + "_" + std::to_string(this->now().nanoseconds())) : goal->task_id;
  task.type = taskTypeFromString(goal->task_type);
  task.goal_params = goalToParams(*goal);
  task.state = TaskState::RECEIVED;
  task.created_time = this->now();

  std::string error_message;
  task.state = TaskState::VALIDATING;
  if (!expandTaskToSteps(
      task, dock_database_path_, staging_x_offset_, staging_yaw_offset_, error_message))
  {
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = false;
    result->error_code = "VALIDATION_FAILED";
    result->message = error_message;
    goal_handle->abort(result);
    return;
  }

  task.state = TaskState::QUEUED;

  {
    std::lock_guard<std::mutex> lock(pending_handles_mutex_);
    pending_goal_handles_[task.task_id] = goal_handle;
  }

  RCLCPP_INFO(
    this->get_logger(), "Queued task '%s' (%s), %zu step(s)",
    task.task_id.c_str(), taskTypeToString(task.type).c_str(), task.steps.size());
  queue_.push(std::move(task));
}

// ==================== Worker thread ====================

void TaskManagerNode::workerLoop()
{
  rclcpp::Rate poll_rate(10.0);

  while (running_.load() && rclcpp::ok()) {
    auto maybe_task = queue_.pop();
    if (!maybe_task) {
      poll_rate.sleep();
      continue;
    }

    Task task = std::move(*maybe_task);

    std::shared_ptr<GoalHandleExecuteTask> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_handles_mutex_);
      auto it = pending_goal_handles_.find(task.task_id);
      if (it != pending_goal_handles_.end()) {
        goal_handle = it->second;
        pending_goal_handles_.erase(it);
      }
    }

    if (!goal_handle) {
      RCLCPP_ERROR(
        this->get_logger(), "No goal handle found for task '%s' -- dropping.", task.task_id.c_str());
      continue;
    }

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<ExecuteTask::Result>();
      result->success = false;
      result->error_code = "CANCELED";
      result->message = "Canceled while queued";
      goal_handle->canceled(result);
      continue;
    }

    task.state = TaskState::EXECUTING;
    {
      std::lock_guard<std::mutex> lock(current_mutex_);
      current_task_ = task;
      current_goal_handle_ = goal_handle;
    }

    auto feedback_msg = std::make_shared<ExecuteTask::Feedback>();
    feedback_msg->task_id = task.task_id;
    auto is_canceling = [goal_handle]() {return goal_handle->is_canceling();};

    {
      std::lock_guard<std::mutex> lock(step_context_mutex_);
      step_context_ = StepContext();
      step_context_.task_id = task.task_id;
      step_context_.task_type = taskTypeToString(task.type);
      step_context_.dock_id = task.goal_params.dock_id;
      step_context_.total_steps = static_cast<int32_t>(task.steps.size());
      last_log_substep_.clear();
    }
    publishStepStatus("TASK", "RUNNING", "Task started");

    bool all_ok = true;
    std::string final_error_code, final_error_message;

    for (size_t i = 0; i < task.steps.size(); ++i) {
      auto & step = task.steps[i];

      // MONITOR_CHARGING reports TaskState::MONITORING instead of
      // EXECUTING -- executor_->isMonitoring() flips true only while
      // that specific step is actually running.
      const bool is_monitor_step = (step.type == StepType::MONITOR_CHARGING);
      const std::string step_state_str = is_monitor_step ?
        taskStateToString(TaskState::MONITORING) : taskStateToString(TaskState::EXECUTING);

      feedback_msg->task_type = taskTypeToString(task.type);
      feedback_msg->current_step = stepTypeToString(step.type);
      feedback_msg->step_index = static_cast<int32_t>(i);
      feedback_msg->total_steps = static_cast<int32_t>(task.steps.size());
      feedback_msg->state = step_state_str;
      feedback_msg->progress = task.steps.empty() ? 100.0f :
        (100.0f * static_cast<float>(i) / static_cast<float>(task.steps.size()));
      feedback_msg->message = "Executing " + stepTypeToString(step.type);
      goal_handle->publish_feedback(feedback_msg);

      {
        std::lock_guard<std::mutex> lock(step_context_mutex_);
        step_context_.step_index = static_cast<int32_t>(i);
        step_context_.attempt = 1;
        step_context_.max_attempts = step.max_retries + 1;
        step_context_.lift_height = step.params.lift_height;
        if (!step.params.dock_id.empty()) {step_context_.dock_id = step.params.dock_id;}
        // Gates the docking_server log lines: they belong to this step.
        step_context_.docking =
          (step.type == StepType::DOCK || step.type == StepType::UNDOCK);
        last_log_substep_.clear();
      }
      publishStepStatus(
        stepTypeToString(step.type), "RUNNING", "Executing " + stepTypeToString(step.type));

      {
        std::lock_guard<std::mutex> lock(current_mutex_);
        current_task_->current_step_index = i;
        current_task_->state = is_monitor_step ? TaskState::MONITORING : TaskState::EXECUTING;
      }

      std::string error_code, error_message;
      bool step_ok = false;

      for (int attempt = 0; attempt <= step.max_retries; ++attempt) {
        if (goal_handle->is_canceling()) {
          error_code = "CANCELED";
          break;
        }
        {
          std::lock_guard<std::mutex> lock(step_context_mutex_);
          step_context_.attempt = attempt + 1;
        }
        step_ok = executor_->executeStep(step, is_canceling, error_code, error_message);
        if (step_ok || error_code == "CANCELED") {
          break;
        }
        RCLCPP_WARN(
          this->get_logger(), "Step %s failed (attempt %d/%d): %s -- retrying",
          stepTypeToString(step.type).c_str(), attempt + 1, step.max_retries + 1,
          error_message.c_str());
        publishStepStatus(
          stepTypeToString(step.type), "RETRY", error_message, error_code);
        step.retry_count++;
      }

      {
        std::lock_guard<std::mutex> lock(step_context_mutex_);
        step_context_.docking = false;
      }

      if (step_ok) {
        publishStepStatus(
          stepTypeToString(step.type), "SUCCESS", stepTypeToString(step.type) + " done");
      } else {
        publishStepStatus(
          stepTypeToString(step.type), error_code == "CANCELED" ? "CANCELED" : "FAILED",
          error_message, error_code);
      }

      if (!step_ok) {
        all_ok = false;
        final_error_code = error_code;
        final_error_message = "Step " + stepTypeToString(step.type) + " failed: " + error_message;
        break;
      }
    }

    auto result = std::make_shared<ExecuteTask::Result>();

    if (all_ok) {
      feedback_msg->state = taskStateToString(TaskState::COMPLETED);
      feedback_msg->progress = 100.0f;
      feedback_msg->message = "Task completed";
      goal_handle->publish_feedback(feedback_msg);

      result->success = true;
      result->error_code = "";
      result->message = "Task '" + task.task_id + "' completed successfully";
      {
        std::lock_guard<std::mutex> lock(step_context_mutex_);
        step_context_.step_index = step_context_.total_steps;
      }
      publishStepStatus("TASK", "SUCCESS", result->message);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Task '%s' completed", task.task_id.c_str());
    } else {
      result->success = false;
      result->error_code = final_error_code;
      result->message = final_error_message;
      publishStepStatus(
        "TASK", final_error_code == "CANCELED" ? "CANCELED" : "FAILED",
        final_error_message, final_error_code);

      if (final_error_code == "CANCELED") {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Task '%s' canceled", task.task_id.c_str());
      } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(
          this->get_logger(), "Task '%s' failed: %s", task.task_id.c_str(),
          final_error_message.c_str());
      }
    }

    {
      std::lock_guard<std::mutex> lock(current_mutex_);
      current_task_.reset();
      current_goal_handle_.reset();
    }
  }
}

// ==================== Services / status topic ====================

void TaskManagerNode::handleGetTaskStatus(
  const std::shared_ptr<GetTaskStatus::Request>, std::shared_ptr<GetTaskStatus::Response> res)
{
  std::lock_guard<std::mutex> lock(current_mutex_);
  if (current_task_) {
    res->has_current_task = true;
    res->current_task_id = current_task_->task_id;
    res->current_task_type = taskTypeToString(current_task_->type);
    res->current_state = taskStateToString(current_task_->state);
    res->current_step_index = static_cast<int32_t>(current_task_->current_step_index);
    res->total_steps = static_cast<int32_t>(current_task_->steps.size());
  } else {
    res->has_current_task = false;
  }
  res->queue_length = static_cast<int32_t>(queue_.size());
}

void TaskManagerNode::handleClearQueue(
  const std::shared_ptr<ClearQueue::Request>, std::shared_ptr<ClearQueue::Response> res)
{
  res->tasks_cleared = queue_.clear();
  res->success = true;
  RCLCPP_INFO(this->get_logger(), "Cleared %d pending task(s) from queue", res->tasks_cleared);
}

void TaskManagerNode::onLogMessage(const rcl_interfaces::msg::Log::SharedPtr log)
{
  if (log->name != docking_logger_name_) {return;}
  {
    // Only while this node is actually running a DOCK/UNDOCK step --
    // docking triggered by anything else isn't part of a task.
    std::lock_guard<std::mutex> lock(step_context_mutex_);
    if (!step_context_.docking) {return;}
  }

  const std::string & text = log->msg;
  std::string step, state;
  if (text.find("Lost detection or did not detect") != std::string::npos) {
    step = "DETECT";
    state = "RETRY";
  } else if (text.find("Failed initial dock detection") != std::string::npos) {
    step = "DETECT";
    state = "FAILED";
  } else if (text.find("Successful initial dock detection") != std::string::npos) {
    step = "DETECT";
    state = "SUCCESS";
  } else if (text.find("Made contact with dock") != std::string::npos) {
    step = "CONTACT";
    state = "SUCCESS";
  } else if (text.find("Robot is charging") != std::string::npos) {
    step = "CHARGING";
    state = "SUCCESS";
  } else if (text.find("Attempting to undock") != std::string::npos) {
    step = "UNDOCK";
    state = "RUNNING";
  } else if (text.find("Robot has reached staging pose") != std::string::npos) {
    step = "UNDOCK_STAGING";
    state = "SUCCESS";
  } else if (text.find("Robot has undocked") != std::string::npos) {
    step = "UNDOCK";
    state = "SUCCESS";
  } else {
    return;
  }

  // "Lost detection" is logged every control cycle (~50 Hz in the sample
  // logs) -- publish transitions, not every repeat.
  const std::string key = step + state;
  {
    std::lock_guard<std::mutex> lock(step_context_mutex_);
    if (key == last_log_substep_) {return;}
    last_log_substep_ = key;
    if (step == "DETECT" && state == "SUCCESS") {step_context_.detected = true;}
    if (step == "CHARGING") {step_context_.charging = true;}
  }
  publishStepStatus(step, state, text);
}

void TaskManagerNode::publishStepStatus(
  const std::string & step, const std::string & state, const std::string & message,
  const std::string & error_code)
{
  std::lock_guard<std::mutex> lock(step_context_mutex_);
  auto msg = forklift_interfaces::msg::TaskStepStatus();
  msg.header.stamp = this->now();
  msg.task_id = step_context_.task_id;
  msg.task_type = step_context_.task_type;
  msg.step = step;
  msg.state = state;
  msg.step_index = step_context_.step_index;
  msg.total_steps = step_context_.total_steps;
  msg.progress = step_context_.total_steps == 0 ? 100.0f :
    (100.0f * static_cast<float>(step_context_.step_index) /
    static_cast<float>(step_context_.total_steps));
  msg.attempt = step_context_.attempt;
  msg.max_attempts = step_context_.max_attempts;
  msg.dock_id = step_context_.dock_id;
  msg.lift_height = static_cast<float>(step_context_.lift_height);
  msg.detected = step_context_.detected;
  msg.charging = step_context_.charging;
  msg.success = (step == "TASK" && state == "SUCCESS");
  msg.error_code = error_code;
  msg.message = message;
  step_status_pub_->publish(msg);
}

void TaskManagerNode::publishStatus()
{
  auto msg = forklift_interfaces::msg::TaskManagerStatus();
  msg.header.stamp = this->now();

  std::lock_guard<std::mutex> lock(current_mutex_);
  if (current_task_) {
    msg.has_current_task = true;
    msg.current_task_id = current_task_->task_id;
    msg.current_task_type = taskTypeToString(current_task_->type);
    msg.current_state = taskStateToString(current_task_->state);
  } else {
    msg.has_current_task = false;
  }
  msg.queue_length = static_cast<int32_t>(queue_.size());

  status_pub_->publish(msg);
}

}  // namespace forklift_task_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(forklift_task_manager::TaskManagerNode)