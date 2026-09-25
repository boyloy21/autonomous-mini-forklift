#include "forklift_task_manager/task_queue.hpp"

#include <algorithm>

namespace forklift_task_manager
{

void TaskQueue::push(Task task)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push_back(std::move(task));
}

std::optional<Task> TaskQueue::pop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  Task front = std::move(queue_.front());
  queue_.pop_front();
  return front;
}

bool TaskQueue::remove(const std::string & task_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = std::find_if(
    queue_.begin(), queue_.end(),
    [&task_id](const Task & t) {return t.task_id == task_id;});
  if (it == queue_.end()) {
    return false;
  }
  queue_.erase(it);
  return true;
}

size_t TaskQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

bool TaskQueue::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.empty();
}

int TaskQueue::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int count = static_cast<int>(queue_.size());
  queue_.clear();
  return count;
}

}  // namespace forklift_task_manager