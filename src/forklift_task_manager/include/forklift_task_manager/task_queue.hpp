#ifndef FORKLIFT_TASK_MANAGER__TASK_QUEUE_HPP_
#define FORKLIFT_TASK_MANAGER__TASK_QUEUE_HPP_

#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "forklift_task_manager/task.hpp"

namespace forklift_task_manager
{

// Simple thread-safe FIFO. The executing worker thread and the action
// server's goal callbacks both touch this concurrently, hence the mutex
// on every operation. Deliberately no condition-variable wait built in
// here -- the worker thread (in TaskManagerNode) polls with a short sleep
// instead, keeping this class dead simple.
class TaskQueue
{
public:
  void push(Task task);

  // Removes and returns the front task, or nullopt if empty.
  std::optional<Task> pop();

  // Removes a specific pending (not-yet-started) task by id. Returns
  // true if found and removed.
  bool remove(const std::string & task_id);

  size_t size() const;
  bool empty() const;

  // Clears all pending tasks, returns how many were removed.
  int clear();

private:
  mutable std::mutex mutex_;
  std::deque<Task> queue_;
};

}  // namespace forklift_task_manager

#endif  // FORKLIFT_TASK_MANAGER__TASK_QUEUE_HPP_