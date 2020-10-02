#pragma once

#include "MotorControlBase.h"
#include <algorithm>

namespace TeensyStep
{

    template <typename Accelerator, typename TimerField>
    class StepControlBase : public MotorControlBase<TimerField>
    {
     public:
        StepControlBase(unsigned pulseWidth = 5, unsigned accUpdatePeriod = 5000);

        // Non-blocking movements ------------------------------------------

        // Move variadic param list of stepper references e.g. moveAsync(m1,m2,m3,m5)
        template <typename... Steppers>
        void moveAsync(Steppers&... steppers) { moveAsync(1.0f, steppers...); }

        // Move variadic param list of stepper references with speed override e.g. moveAsync(0.2f,m1,m2,m3,m5)
        template <typename... Steppers>
        void moveAsync(float speedOverride, Steppers&... steppers);

        // Move array of stepper pointers with speed override
        template <size_t N>
        void moveAsync(float speedOverride, Stepper* (&steppers)[N]);

        // Move array of stepper pointers
        template <size_t N>
        void moveAsync(Stepper* (&steppers)[N]) { moveAsync(1.0f, steppers); }

        // non blocking stop command
        void stopAsync();

        // Blocking movements ----------------------------------------------

        // Move variadic param list of stepper references e.g. move(m1,m2,m3,m5)
        template <typename... Steppers>
        void move(Steppers&... steppers) { move(1.0f, steppers...); }

        // Move variadic param list of stepper references with speed override. E.g., move(0.5f, m1,m2,m3,m5)
        template <typename... Steppers>
        void move(float speedOverride, Steppers&... steppers);

        // Move array of stepper pointers
        template <size_t N>
        void move(Stepper* (&steppers)[N]) { move(1.0f, steppers); }

        // Move array of stepper pointers with speed override
        template <size_t N>
        void move(float speedOverride, Stepper* (&steppers)[N]);

        // blocking stop command
        void stop();

        // Misc ---------------------------------------------------------

        // set callback function to be called when target is reached
        void setCallback(void (*_callback)()) { this->callback = _callback; }

     protected:
        void accTimerISR();

        void doMove(int N, float speedOverride = 1.0f, bool startTimers = true);

        Accelerator accelerator;

        StepControlBase(const StepControlBase&) = delete;
        StepControlBase& operator=(const StepControlBase&) = delete;
    };

    // Implementation *************************************************************************************************

    template <typename a, typename t>
    StepControlBase<a, t>::StepControlBase(unsigned pulseWidth, unsigned accUpdatePeriod)
        : MotorControlBase<t>(pulseWidth, accUpdatePeriod)
    {
        this->mode = MotorControlBase<t>::Mode::target;
    }

    template <typename a, typename t>
    void StepControlBase<a, t>::doMove(int N, float speedOverride, bool startTimers)
    {
        //Calculate Bresenham parameters -------------------------------------
        std::sort(this->motorList, this->motorList + N, Stepper::cmpDelta); // The motor which does most steps leads the movement, move to top of list
        this->leadMotor = this->motorList[0];
       // Serial.printf("Stepper: %s is the leader\r\n", this->leadMotor->name.c_str());

        for (int i = 1; i < N; i++)
        {
            this->motorList[i]->B = 2 * this->motorList[i]->A - this->leadMotor->A;
        }

        // Calculate acceleration parameters --------------------------------
        uint32_t targetSpeed = std::abs((*std::min_element(this->motorList, this->motorList + N, Stepper::cmpVmin))->vMax) * speedOverride; // use the lowest max frequency for the move, scale by relSpeed
        uint32_t pullInSpeed = this->leadMotor->vPullIn;
        uint32_t pullOutSpeed = this->leadMotor->vPullOut;
        uint32_t acceleration = (*std::min_element(this->motorList, this->motorList + N, Stepper::cmpAcc))->a; // use the lowest acceleration for the move
/*
        if (this->leadMotor->A == 0 || targetSpeed == 0){
            // TODO problem here, if abs positions are used and same position is end and start in motion chain it will abort here and never continue the chain.
            // removing this exit causes setAbsPosition to hack 1 step on call.
            // must have additional condition here that checks all targets if any has more movements and start in that case.
            //Serial.println("Aborting as we already are at desired position");
            return;
        }
*/

        // Start move--------------------------
        // it's important that prepareMovement doesn't return vs = 0 here when running a motion as it will cause  the stepper interrupt to end and the timers won't restart
        // This seems to not happen when running in the old target mode as the stop is returned before the timers are started.
        // A  workaround for now  is to never use vs = 0 in a motion chain.
        this->timerField.setStepFrequency(accelerator.prepareMovement(this->leadMotor->current, this->leadMotor->target, targetSpeed, pullInSpeed, pullOutSpeed, acceleration));
        if(startTimers){
            this->timerField.begin();
            this->timerField.stepTimerStart();
            this->timerField.accTimerStart();
        }
    }

    // ISR -----------------------------------------------------------------------------------------------------------

    template <typename a, typename t>
    void StepControlBase<a, t>::accTimerISR()
    {
        if(!this->isRunning())
        {
            return;
        }
        int32_t speed = accelerator.updateSpeed(this->leadMotor->current);
        // Movement is finished when speed is 0
        if(speed == 0)
        {
            // Serial.printf("End of movement to relative pos: %d\r\n", this->leadMotor->target);
            bool any_targets = false;
            int N = 0; // no storage of current attached steppers so need to store it here for doMove call

            // check if any steppers have more movement enqueued
            while(this->motorList[N] != nullptr)
            {
                //int32_t current_pos = this->motorList[N]->current;
                any_targets |= this->motorList[N]->nextTarget();
                //Serial.printf("'%s' has segment, current pos: %d target %d, pullin: %d\r\n", this->motorList[N]->name.c_str(), current_pos, this->motorList[N]->target, this->motorList[N]->vPullIn);
                N++;
            }
            if(any_targets)
            {
                doMove(N, 1.0f, false);
                return;
            }
        }
        this->timerField.setStepFrequency(speed);
    }

    // Non blocking movements ---------------------------------------------------------------------------------------

    template <typename a, typename t>
    template <typename... Steppers>
    void StepControlBase<a, t>::moveAsync(float speedOverride, Steppers&... steppers)
    {
        this->attachStepper(steppers...);
        doMove(sizeof...(steppers), speedOverride);
    }

    template <typename a, typename t>
    template <size_t N>
    void StepControlBase<a, t>::moveAsync(float speedOverride, Stepper* (&motors)[N]) //move up to maxMotors motors synchronously
    {
        this->attachStepper(motors);
        doMove(N, speedOverride);
    }

    template <typename a, typename t>
    void StepControlBase<a, t>::stopAsync()
    {
        if (this->isRunning())
        {
            uint32_t newTarget = accelerator.initiateStopping(this->leadMotor->current);
            this->leadMotor->target = this->leadMotor->current + this->leadMotor->dir * newTarget;
        }
    }

    // Blocking movmenents -------------------------------------------------------------------------------------------------

    template <typename a, typename t>
    template <typename... Steppers>
    void StepControlBase<a, t>::move(float speedOverride, Steppers&... steppers)
    {
        moveAsync(speedOverride, steppers...);
        while (this->timerField.stepTimerIsRunning())
        {
            delay(1);
        }
    }

    template <typename a, typename t>
    template <size_t N>
    void StepControlBase<a, t>::move(float speedOverride, Stepper* (&motors)[N])
    {
        moveAsync(speedOverride, motors);
        while (this->isRunning())
        {
            delay(1);
        }
    }

    template <typename a, typename t>
    void StepControlBase<a, t>::stop()
    {
        stopAsync();
        while (this->isRunning())
        {
            delay(1);
        }
    }
}
