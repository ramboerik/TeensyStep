#pragma once

#include <Arduino.h>
#include "TickTimer.h"
#include "../TF_Handler.h"

class TickTimerField
{
  public:
    inline TickTimerField(TeensyStep::TF_Handler *);

    inline bool begin();
    inline void end();
    inline void endAfterPulse() { lastPulse = true; }

    inline void stepTimerStart() { stepTimer.start(); }
    inline void stepTimerStop() { stepTimer.stop(); }
    inline bool stepTimerIsRunning() const { return stepTimer.isRunning(); }
    inline void setStepFrequency(unsigned f) {stepTimer.setFrequency(f); }
    inline unsigned getStepFrequency() { return 0; }

    inline void accTimerStart() { accTimer.start(); }
    inline void accTimerStop() { accTimer.stop(); }
    inline void setAccUpdatePeriod(unsigned period) { accTimer.setPeriod(period); }

    inline void setPulseWidth(unsigned delay) { delayTimer.setDelay(delay); }
    inline void triggerDelay() { delayTimer.start(); }

  protected:
    TeensyStep::TF_Handler *handler;

    PeriodicTimer stepTimer, accTimer;
    OneShotTimer delayTimer;
    bool lastPulse = false;
    int starts =0, ends = 0, accs = 0;
};

// IMPLEMENTATION ====================================================================

TickTimerField::TickTimerField(TeensyStep::TF_Handler *_handler)
    : handler(_handler),
      stepTimer([this] { handler->stepTimerISR(); starts++; }, "step"),
      //stepTimer(test),
      accTimer([this] { handler->accTimerISR(); accs++; }, "acc"),
      delayTimer([this] {
          handler->pulseTimerISR();
         // Serial.println("END OF PUSLE HERE");
         ends++;
          if(lastPulse)
          {
              //Serial.printf("DONE: number of accs: %d, starts: %d, ends: %d\r\n", accs, starts, ends);
              end();
          }
      }, 0, "delay")
{
    TimerControl::attachTimer(&stepTimer);
    TimerControl::attachTimer(&accTimer);
    TimerControl::attachTimer(&delayTimer);
}



bool TickTimerField::begin()
{
    TimerControl::begin();
    lastPulse = false;
    return true;
}

void TickTimerField::end()
{
    stepTimer.stop();
    accTimer.stop();
}
