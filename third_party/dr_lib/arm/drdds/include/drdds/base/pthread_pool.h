#ifndef PTHREAD_POOL_H_
#define PTHREAD_POOL_H_
#include "drdds/base/task.h"

namespace DrDDS {
class PthreadPool
{
  public:
  PthreadPool(int min, int max);
  ~PthreadPool();
  void AddTask(Task *task);
  int GetBusyNum();
  int GetLiveNum();

  private:
  static void *ManagerPool(void *arg);
  static void *WorkPool(void *arg);
  void threadExit();

  private:
  static const int add_number_;
  TaskQueue *task_queue_;
  pthread_t *workers_;
  pthread_t manager_;
  int max_pthread_num_;
  int min_pthread_num_;
  int busy_pthread_num_;
  int live_pthread_num_;
  int exit_pthread_num_;
  pthread_mutex_t lock_;
  pthread_cond_t queue_not_empty_cond_;
  bool is_shut_down_;
};
} // namespace DrDDS

#endif // THREADPOOL_H
