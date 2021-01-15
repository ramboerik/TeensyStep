#if defined(__linux__)
#pragma once

#include <Arduino.h>
#include <functional>
#include <limits>
#include <vector>
#include <algorithm>

#include <chrono>

using namespace std::chrono;
using callback_t = std::function<void(void)>;

class TimerControl;

class TimerBase
{
  public:
    TimerBase(callback_t cb, bool _isPeriodic, const char*_name = "")
    {
        callback = cb;
        run = false;
        isPeriodic = _isPeriodic;
        unique_id = shared_id++;
        snprintf(name, sizeof(name), "%s", _name);
    };

    static microseconds get_us() {
        return duration_cast<microseconds>(system_clock::now().time_since_epoch());
    }

    bool operator==(const TimerBase& other) const { return unique_id == other.unique_id; }

    inline void start() { startCnt = TimerBase::get_us(); run = true; }
    inline void stop() { run = false; }
    inline bool isRunning() const { return run; }

  protected:
    microseconds deltaCnt, startCnt;
    bool isPeriodic;
    bool run;
    callback_t callback;
    static int shared_id;
    int unique_id;
    char name[32];

    friend TimerControl;
};

class PeriodicTimer : public TimerBase
{
  public:
    PeriodicTimer(callback_t cb, const char* name = "") : TimerBase(cb, true, name)
    {

    }

    inline void setFrequency(float hz)
    {
        deltaCnt = microseconds(hz == 0 ? 10 : unsigned(1000000 / hz)); // running in computes native speed, varies alot depending on
                                                                        // other cpu load.
    }

    inline void setPeriod(uint32_t usec)
    {
        deltaCnt = microseconds(usec);
    }

    static float minFrequency;
};

class OneShotTimer : public TimerBase
{
  public:
    OneShotTimer(callback_t cb, unsigned delay = 0, const char* name = "") : TimerBase(cb, false, name)
    {
        setDelay(delay);
    }

    void setDelay(unsigned usec)
    {
        deltaCnt = microseconds(usec);//F_CPU / 1'000'000 * microSeconds;
    }
};

class TimerControl
{
  public:
    static void begin() { }

    static void attachTimer(TimerBase *timer) {
        getTimers().push_back(timer);
    }

    static void detachTimer(TimerBase *timer)
    {
        auto timers = getTimers();
        for (auto it = timers.begin(); it != timers.end(); ) {
            if (**it == *timer) {
                it = timers.erase(it);
                break;
            }
        }
    }

    static inline void tick()
    {
        auto& timers = getTimers();

        // run the interrupts in the same order they would have been triggered if they were real interrupts.
        std::sort(timers.begin(), timers.end(), [] (TimerBase *t1, TimerBase *t2) {
            return t1->startCnt + t1->deltaCnt < t2->startCnt + t2->deltaCnt;
        });

        microseconds now = TimerBase::get_us(); // must be outside the loop or otherwise the interrupt order
                                                // may be different than real interrupts.
        for(auto& timer : timers) {

            if (timer->run && (now - timer->startCnt >= timer->deltaCnt)) {
            //    Serial.printf("Fired timer %s. trigger: %lu, delayed: %lu\r\n", timer->name, timer->startCnt, now - timer->startCnt - timer->deltaCnt);
                timer->callback();

                if (timer->isPeriodic)
                    timer->startCnt = TimerBase::get_us();
                else
                    timer->run = false;
           }
       }
    }

    static std::vector<TimerBase*>& getTimers() {
        static std::vector<TimerBase*> timers;
        return timers;
    }
};

#endif
