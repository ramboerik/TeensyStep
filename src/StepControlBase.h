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
        Stepper* fullList[MaxMotors + 1];

        void accTimerISR();

        bool nextTarget();
        bool filterWork(int &len);
        bool doMove(float speedOverride = 1.0f, bool startTimers = true);
        int getNumSteppers();

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
    int StepControlBase<a, t>::getNumSteppers(){
        int len = 0;
        while(this->fullList[len++] != nullptr);
        return len - 1;
    }

    /**
     * \brief Load the next target for all steppers.
     * \return True if there are targets left for any stepper, false if there is nothing more to do.
     */
    template <typename a, typename t>
    bool StepControlBase<a, t>::nextTarget(){
        bool anyTarget = false;
        int i = 0;
        // check if any steppers have more movement enqueued
        while(this->fullList[i] != nullptr)
        {
            anyTarget |= this->fullList[i++]->nextTarget();
        }
        return anyTarget;
    }

    /**
     * \brief Iterate attached steppers and generate a filtered list that only contains
     *        the steppers that will move. Steppers that already are at their target or have
     *        zero speed will be filtered out. This is an optimization of cases when
     *        steppers with slow target speed otherwise would determine/limit the max speed for all
     *        other steppers during the move.
     * \param[out] Resulting list of steppers, Must be of length MaxMotors + 1. The last element
     *             in the  list will be nullptr.
     * \param[out] Number of steppers in resulting list.
     */
    template <typename a, typename t>
    bool StepControlBase<a, t>::filterWork(int &len)
    {
        len = 0;
        int maxSteppers = getNumSteppers();
        for (int i = 0; i < maxSteppers; i++)
        {
            if(this->fullList[i]->vMax == 0 || this->fullList[i]->A == 0)
            {
                //Serial.printf("Removing stepper index: %d, %s with vMax: %d, steps: %d\r\n", i, this->motorList[i]->name.c_str(), this->motorList[i]->vMax, this->motorList[i]->A);
                continue;
            }
            //Serial.printf("Stepper %s has work, vMax: %d, steps: %d\r\n", this->motorList[i]->name.c_str(), this->motorList[i]->vMax, this->motorList[i]->A);
            this->motorList[len++] = this->fullList[i];
        }
        this->motorList[len] = nullptr;
        return len != 0;
    }

    template <typename a, typename t>
    bool StepControlBase<a, t>::doMove(float speedOverride, bool startTimers)
    {
        int N = 0;

        // Search next target with work if the current loaded target doesn't have any.
        // Duplicated targets and targets will zero speed/distance will be removed.
        while(!filterWork(N))
        {
            if(!nextTarget())
            {
                // no more work or targets, time to end movement.
                return false;
            }
        }

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

        // Start move--------------------------
        // it's important that prepareMovement doesn't return vs = 0 here when running a motion as it will cause the stepper interrupt to end and the timers won't restart
        // This seems to not happen when running in the old target mode as the stop is returned before the timers are started.
        // A  workaround for now  is to never use vs = 0 in a motion chain.
        this->timerField.setStepFrequency(accelerator.prepareMovement(this->leadMotor->current, this->leadMotor->target, targetSpeed, pullInSpeed, pullOutSpeed, acceleration));
        if(startTimers){
            this->timerField.begin();
            this->timerField.stepTimerStart();
            this->timerField.accTimerStart();
        }
        return true;
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
            nextTarget(); // load next move to be processed
            if(doMove(1.0f, false))
            {
                // If there are more targets end here as the step freq. timer
                // has already been updated in the doMove call.
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
        int i = 0;
        while(this->motorList[i] != nullptr){
            fullList[i] = this->motorList[i];
            i++;
        }
        this->fullList[i] = nullptr;
        doMove(speedOverride);
    }

    template <typename a, typename t>
    template <size_t N>
    void StepControlBase<a, t>::moveAsync(float speedOverride, Stepper* (&motors)[N]) //move up to maxMotors motors synchronously
    {
        this->attachStepper(motors);
        int i = 0;
        while(this->motorList[i] != nullptr){
            this->fullList[i] = this->motorList[i];
            i++;
        }
        this->fullList[i] = nullptr;
        doMove(speedOverride);
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
