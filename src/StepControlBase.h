#pragma once

#include "./TeensyDelay/TeensyDelay.h"
#include "MotorControlBase.h"
#include "Stepper.h"

#include <algorithm>

template <typename Accelerator, typename Timer, unsigned pulseWidth, unsigned accUpdatePeriod>
class StepControlBase : public MotorControlBase, public TF_Handler
{
  public:  
    StepControlBase() { OK = st.begin(this); }
    ~StepControlBase() { st.end(); }

    // Blocking movements --------------------
    template <typename... Steppers>
    void move(Steppers &... steppers);

    template <size_t N>
    void move(Stepper *(&motors)[N]);

    void stop();

    // Non-blocking movements ----------------
    void moveAsync(Stepper &stepper);

    template <typename... Steppers>
    void moveAsync(Stepper &stepper, Steppers &... steppers);

    template <size_t N>
    void moveAsync(Stepper *(&motors)[N]);
    void stopAsync();

    // Misc
    void setCallback(void (*_callback)()) { callback = _callback; }

  protected:
    // Construction
    StepControlBase(const StepControlBase &) = delete;
    StepControlBase &operator=(const StepControlBase &) = delete;

    void stepTimerISR();
    void accTimerISR() {}
    void delayTimerISR(){};

    void delayISR(unsigned channel);

    void (*callback)() = nullptr;

    void doMove(int N, bool mode = true);

    Accelerator accelerator;
    Timer st;
};

// Implementation *************************************************************************************************

template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::doMove(int N, bool move)
{
    //Calculate Bresenham parameters ----------------------------------------------------------------
    std::sort(motorList, motorList + N, Stepper::cmpDelta); // The motor which does most steps leads the movement, move to top of list
    leadMotor = motorList[0];

    for (int i = 1; i < N; i++)
    {
        motorList[i]->B = 2 * motorList[i]->A - leadMotor->A;
    }

    // Calculate acceleration parameters --------------------------------------------------------------
    uint32_t targetSpeed = std::abs((*std::min_element(motorList, motorList + N, Stepper::cmpVmin))->vMax); // use the lowest max frequency for the move, scale by relSpeed
    uint32_t acceleration = (*std::min_element(motorList, motorList + N, Stepper::cmpAcc))->a;              // use the lowest acceleration for the move
    if (leadMotor->A == 0 || targetSpeed == 0)
        return;

    // Start move---------------------------------------------------------------------------------------
    st.stepTimerStop();
    st.stepTimerSetFrequency(accelerator.prepareMovement(leadMotor->current, leadMotor->target, targetSpeed, acceleration));
    st.stepTimerStart();

    stepTimerISR();                // initiate first step immediately (no need to wait for the potentially long first cycle)
    delayISR(accLoopDelayChannel); // implicitely start the accLoop
}

// ISR -----------------------------------------------------------------------------------------------------------
template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::stepTimerISR()
{
    Stepper **slave = motorList;
    leadMotor->doStep(); // move master motor

    while (*(++slave) != nullptr) // move slave motors if required (https://en.wikipedia.org/wiki/Bresenham)
    {
        if ((*slave)->B >= 0)
        {
            (*slave)->doStep();
            (*slave)->B -= leadMotor->A;
        }
        (*slave)->B += (*slave)->A;
    }
    TeensyDelay::trigger(p, pinResetDelayChannel); // start delay line to dactivate all step pins

    if (leadMotor->current == leadMotor->target) // stop timer and call callback if we reached target
    {
        st.stepTimerStop();
        if (callback != nullptr)
            callback();
    }
}

template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::delayISR(unsigned channel)
{
    //clear all step pins ----------------------------------------------
    if (channel == pinResetDelayChannel)
    {
        Stepper **motor = motorList;
        while ((*motor) != nullptr)
        {
            (*motor++)->clearStepPin();
        }
    }

    // calculate new speed  --------------------------------------------
    if (channel == accLoopDelayChannel)
    {
        if (st.stepTimerIsRunning())
        {
            noInterrupts();
            TeensyDelay::trigger(u, accLoopDelayChannel); // retrigger
            interrupts();

            st.stepTimerSetFrequency(accelerator.updateSpeed(leadMotor->current));
        }
    }
}

// MOVE ASYNC Commands -------------------------------------------------------------------------------------------------

template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::moveAsync(Stepper &stepper)
{
    motorList[mCnt++] = &stepper;
    motorList[mCnt] = nullptr;
    doMove(mCnt);
    mCnt = 0;
}

template <typename a, typename t, unsigned p, unsigned u>
template <typename... Steppers>
void StepControlBase<a, t, p, u>::moveAsync(Stepper &stepper, Steppers &... steppers)
{
    static_assert(sizeof...(steppers) < MaxMotors, "Too many motors used. Please increase MaxMotors in file MotorControlBase.h");

    motorList[mCnt++] = &stepper;
    moveAsync(steppers...);
}

template <typename a, typename t, unsigned p, unsigned u>
template <size_t N>
void StepControlBase<a, t, p, u>::moveAsync(Stepper *(&motors)[N]) //move up to maxMotors motors synchronously
{
    static_assert((N + 1) <= sizeof(motorList) / sizeof(motorList[0]), "Too many motors used. Please increase MaxMotors in file MotorControlBase.h");

    for (unsigned i = 0; i < N; i++)
    {
        motorList[i] = motors[i];
    }
    motorList[N] = nullptr;
    doMove(N);
}

// MOVE Commands -------------------------------------------------------------------------------------------------

template <typename a, typename t, unsigned p, unsigned u>
template <typename... Steppers>
void StepControlBase<a, t, p, u>::move(Steppers &... steppers)
{
    moveAsync(steppers...);
    while (st.stepTimerIsRunning())
    {
        delay(1);
    }
}

template <typename a, typename t, unsigned p, unsigned u>
template <size_t N>
void StepControlBase<a, t, p, u>::move(Stepper *(&motors)[N])
{
    moveAsync(motors, N);
    while (st.stepTimerIsRunning())
    {
        delay(1);
    }
}

template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::stopAsync()
{
    uint32_t newTarget = accelerator.initiateStopping(leadMotor->current);
    leadMotor->target = newTarget;
}

template <typename a, typename t, unsigned p, unsigned u>
void StepControlBase<a, t, p, u>::stop()
{
    stopAsync();
    while (st.stepTimerIsRunning())
    {
        delay(1);
    }
}
