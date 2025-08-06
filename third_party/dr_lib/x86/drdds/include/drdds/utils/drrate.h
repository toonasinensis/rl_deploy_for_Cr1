#ifndef RATE_H_
#define RATE_H_

#include "drdds/base/timer.h"

class DrRate {
public:
  DrRate(unsigned int hz, __uint8_t priority=80, __uint8_t cpu_id=0);
  ~DrRate();
  bool Run();
  uint64_t Sleep();
private:
  Timer* timer_ {nullptr};
  unsigned int hz_;
};

#endif
