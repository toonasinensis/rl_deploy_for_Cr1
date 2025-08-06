#ifndef TIMER_H_
#define TIMER_H_

#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <stdint.h>

class Timer
{
  public:
  Timer(int ms, __uint8_t priority, __uint8_t cpu_id);
  ~Timer();
  uint64_t WaitForTimerInterrupt();
  void TimerStart();
  void TimerStop();
  void TimerRestart();
  bool TimerEnabled();

  private:
  int time_;
  int timer_fd_{-1};
  pthread_cond_t timer_cond_;
  pthread_mutex_t timer_mutex_;
};

#endif
