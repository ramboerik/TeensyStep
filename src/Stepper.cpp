#include <Arduino.h>
#include "Stepper.h"

namespace TeensyStep
{
    Stepper::Stepper(const int _stepPin, const int _dirPin, const char* name)
        : current(0), t_index(0), stepPin(_stepPin), dirPin(_dirPin), name(name)
    {
        setStepPinPolarity(HIGH);
        setInverseRotation(false);
        setAcceleration(aDefault);
        setMaxSpeed(vMaxDefault);
        setPullInSpeed(vPullInOutDefault);

        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    Stepper& Stepper::setStepPinPolarity(int polarity)
    {
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        // Calculate adresses of bitbanded pin-set and pin-clear registers
        uint32_t pinRegAddr = (uint32_t)digital_pin_to_info_PGM[stepPin].reg; //GPIO_PDOR
        uint32_t* pinSetReg = (uint32_t*)(pinRegAddr + 4 * 32);               //GPIO_PSOR = GPIO_PDOR + 4
        uint32_t* pinClearReg = (uint32_t*)(pinRegAddr + 8 * 32);             //GPIO_PCOR = GPIO_PDOR + 8

        // Assign registers according to step option
        if (polarity == LOW)
        {
            stepPinActiveReg = pinClearReg;
            stepPinInactiveReg = pinSetReg;
        } else
        {
            stepPinActiveReg = pinSetReg;
            stepPinInactiveReg = pinClearReg;
        }
#else
        this->polarity = polarity;
#endif
        clearStepPin(); // set step pin to inactive state
        return *this;
        clearStepPin(); // set step pin to inactive state
        return *this;
    }

    Stepper& Stepper::setInverseRotation(bool reverse)
    {
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        // Calculate adresses of bitbanded pin-set and pin-clear registers
        uint32_t pinRegAddr = (uint32_t)digital_pin_to_info_PGM[dirPin].reg; //GPIO_PDOR
        uint32_t* pinSetReg = (uint32_t*)(pinRegAddr + 4 * 32);              //GPIO_PSOR = GPIO_PDOR + 4
        uint32_t* pinClearReg = (uint32_t*)(pinRegAddr + 8 * 32);            //GPIO_PCOR = GPIO_PDOR + 8

        if (reverse)
        {
            dirPinCwReg = pinClearReg;
            dirPinCcwReg = pinSetReg;
        } else
        {
            dirPinCwReg = pinSetReg;
            dirPinCcwReg = pinClearReg;
        }
#else
        this->reverse = reverse;
#endif
        return *this;
    }

    Stepper& Stepper::setAcceleration(uint32_t a) // steps/s^2
    {
        this->a = std::min(aMax, a);
        return *this;
    }

    Stepper& Stepper::setMaxSpeed(int32_t speed)
    {
        setDir(speed >= 0 ? 1 : -1);
        vMax = std::min(vMaxMax, std::max(-vMaxMax, speed));
        return *this;
    }

    Stepper& Stepper::setPullInSpeed(int32_t speed)
    {
        return setPullInOutSpeed(speed, speed);
    }

    Stepper& Stepper::setPullInOutSpeed(int32_t pullInSpeed, int32_t pullOutSpeed)
    {
        vPullIn = std::min(std::abs(pullInSpeed), vMax);
        vPullOut = std::min(std::abs(pullOutSpeed), vMax);
        return *this;
    }

    void Stepper::setTargetAbs(int32_t target)
    {
        Stepper::setTargetRel(target - current);
    }

    void Stepper::setTargetRel(int32_t delta)
    {
        setDir(delta < 0 ? -1 : 1);
        target = current + delta;
        A = std::abs(delta);
    }

    void Stepper::loadTarget(const Target& t)
    {
        //Serial.printf("Loading target %d, index: %d of total: %d\r\n", t.target, t_index, targets.size());
        if(t.speed != 0) setMaxSpeed(t.speed);
        setPullInOutSpeed(t.vPullIn, t.vPullOut);
        t.abs ? setTargetAbs(t.target) : setTargetRel(t.target);
        t_index++;
    }

    bool Stepper::addTargetAbs(int32_t pos, int32_t speed, int32_t pullIn, int32_t pullOut)
    {
        Target *t = new Target(pos, speed, pullIn, pullOut, true);
        if(!t)
        {
            return false;
        }
        // Make sure the first target is loaded for the move call
        if(targets.size() == 0)
        {
            loadTarget(*t);
        }
        targets.push_back(t);
        return true;
    }

    bool Stepper::addTargetRel(int32_t delta, int32_t speed, int32_t pullIn, int32_t pullOut)
    {
        Target *t = new Target(delta, speed, pullIn, pullOut);
        if(!t)
        {
            return false;
        }
        // Make sure the first target is loaded for the move call
        if(targets.size() == 0)
        {
            loadTarget(*t);
        }
        targets.push_back(t);
        return true;
    }

    void Stepper::repeatTargets()
    {
        t_index = 0;
        // Reload the first target in the list
        if(targets.size() > 0)
        {
            loadTarget(*targets[0]);
        }
    }

    void Stepper::removeTargets(){
        t_index = 0;
        for(auto it = targets.begin(); it != targets.end();)
        {
            it = targets.erase(it);
        }
    }

    bool Stepper::nextTarget()
    {
        if(t_index >= targets.size())
        {
            return false;
        }
        loadTarget(*targets[t_index]);
        return true;
    }
}
