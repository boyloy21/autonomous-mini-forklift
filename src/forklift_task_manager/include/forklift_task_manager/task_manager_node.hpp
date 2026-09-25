#ifndef FORKLIFT_TASK_MANAGER__TASK_MANAGER_NODE_HPP_
#define FORKLIFT_TASK_MANAGER__TASK_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include "forklift_interfaces/action/execute_task.hpp"
#include "forklift_interfaces/srv/get_task_status.hpp"
#include "forklift_interfaces/srv/clear_queue.hpp"
#include "forklift_interfaces/msg/task_manager_status.hpp"
#include "forklift_interfaces/msg/task_step_status.hpp"

#include "forklift_task_manager/task.hpp"
#include "forklift_task_manager/task_queue.hpp"
#include "forklift_task_manager/task_executor.hpp"

namespace forklift_task_manager
{

class TaskManagerNode : public rclcpp::Node
{
public:
  using ExecuteTask = forklift_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using GetTaskStatus = forklift_interfaces::srv::GetTaskStatus;
  using ClearQueue = forklift_interfaces::srv::ClearQueue;

  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskManagerNode() override;

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle);

  void workerLoop();

  void handleGetTaskStatus(
    const std::shared_ptr<GetTaskStatus::Request> req, std::shared_ptr<GetTaskStatus::Response> res);
  void handleClearQueue(
    const std::shared_ptr<ClearQueue::Request> req, std::shared_ptr<ClearQueue::Response> res);
  void publishStatus();

  // One task_status message per step/sub-step transition. `step` is a
  // TaskStepStatus step name (a StepType, a docking sub-step such as
  // DETECT, or "TASK" for the task-level start/finish).
  void publishStepStatus(
    const std::string & step, const std::string & state, const std::string & message,
    const std::string & error_code = "");

  // docking_server reports several phases only through its log output --
  // no message or action feedback field carries them (checked against the
  // installed opennav_docking): "Lost detection or did not detect",
  // "Successful initial dock detection", "Made contact with dock",
  // "Robot is charging!", "Robot has reached staging pose" (undocking)
  // and "Robot has undocked!". This node already owns the DOCK/UNDOCK
  // steps, so it translates those lines into task_status sub-steps here
  // rather than leaving every observer to scrape /rosout itself.
  void onLogMessage(const rcl_interfaces::msg::Log::SharedPtr msg);

  void loadParameters();

  // Populates the shared GoalParams fields from an ExecuteTask::Goal --
  // used both by handleGoal (validation probe) and handleAccepted (the
  // real task), so the two never drift apart.
  static GoalParams goalToParams(const ExecuteTask::Goal & goal);

  std::string dock_database_path_;
  double staging_x_offset_;
  double staging_yaw_offset_;
  double status_publish_rate_hz_;

  TaskQueue queue_;
  std::unique_ptr<TaskExecutor> executor_;

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Service<GetTaskStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<ClearQueue>::SharedPtr clear_queue_srv_;
  rclcpp::Publisher<forklift_interfaces::msg::TaskManagerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<forklift_interfaces::msg::TaskStepStatus>::SharedPtr step_status_pub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr docking_log_sub_;
  std::string docking_logger_name_;
  bool use_docking_logs_{true};
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex current_mutex_;
  std::optional<Task> current_task_;
  std::shared_ptr<GoalHandleExecuteTask> current_goal_handle_;

  std::mutex pending_handles_mutex_;
  std::unordered_map<std::string, std::shared_ptr<GoalHandleExecuteTask>> pending_goal_handles_;

  // Context for publishStepStatus(), owned by the worker thread: which
  // step of which task the executor is currently running.
  struct StepContext
  {
    std::string task_id;
    std::string task_type;
    std::string dock_id;
    double lift_height{0.0};
    int32_t step_index{0};
    int32_t total_steps{0};
    int32_t attempt{1};
    int32_t max_attempts{1};
    bool detected{false};
    bool charging{false};
    // True only while a DOCK/UNDOCK step is running, so docking_server
    // log lines from anything else are ignored.
    bool docking{false};
  };
  StepContext step_context_;
  // publishStepStatus() is called from the worker thread, the executor's
  // DockRobot feedback callback and the log subscription.
  std::mutex step_context_mutex_;
  std::string last_log_substep_;

  std::thread worker_thread_;
  std::atomic<bool> running_{true};
};

}  // namespace forklift_task_manager

#endif  // FORKLIFT_TASK_MANAGER__TASK_MANAGER_NODE_HPP_