#include "TickTimer.h"

float PeriodicTimer::minFrequency = (float)F_CPU / std::numeric_limits<uint32_t>::max();
TimerBase* TimerControl::firstTimer = nullptr;
TimerBase* TimerControl::lastTimer = nullptr;
