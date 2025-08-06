#ifndef DRDDS_TASK_H_
#define DRDDS_TASK_H_

#include <pthread.h>
#include <queue>
#include <functional>

namespace DrDDS {
typedef struct Task
{
  using TaskFunction = std::function<void(void *)>;
  Task()
  {
    function_ = nullptr;
    arg_ = nullptr;
  }
  Task(TaskFunction func, void *arg)
  {
    function_ = func;
    arg_ = arg;
  }

  TaskFunction function_;
  void *arg_;
} Task_t;

class TaskQueue
{
  public:
  TaskQueue();
  ~TaskQueue();
  void AddTask(Task_t *task);
  Task_t *GetTask();
  inline int GetSize() { return tasks_.size(); }

  private:
  std::queue<Task_t *> tasks_;
  pthread_mutex_t task_mutex_;
};
} // namespace DrDDS

#endif
