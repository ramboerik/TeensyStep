#if defined(__linux)
#include "TickTimer.h"

float PeriodicTimer::minFrequency = 1000000 / std::numeric_limits<uint32_t>::max();
int TimerBase::shared_id = 0;
#endif
