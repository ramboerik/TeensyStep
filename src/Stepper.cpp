#include <Arduino.h>
#include "Stepper.h"

namespace TeensyStep
{
    const int32_t Stepper::vMaxMax = 300000;   // largest speed possible (steps/s)
    const uint32_t Stepper::aMax = 500000;     // speed up to 500kHz within 1 s (steps/s^2)
    const uint32_t Stepper::vMaxDefault = 800; // should work with every motor (1 rev/sec in 1/4-step mode)
    const uint32_t Stepper::vPullInOutDefault = 100;
    const uint32_t Stepper::aDefault = 2500; // reasonably low (~0.5s for reaching the default speed)

    Stepper::Stepper(const int _stepPin, const int _dirPin, const char* name)
        : current(0), targetsLen(0), targetsPos(0), stepPin(_stepPin), dirPin(_dirPin)
    {
        setStepPinPolarity(HIGH);
        setInverseRotation(false);
        setAcceleration(aDefault);
        setMaxSpeed(vMaxDefault);
        setPullInOutSpeed(vPullInOutDefault, vPullInOutDefault);

        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        snprintf(this->name, sizeof(this->name), "%s", name);
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
        originalvMax = vMax;
        return *this;
    }

    Stepper& Stepper::setPullInSpeed(int32_t speed)
    {
        vPullIn = std::abs(speed);
        originalvPullIn = vPullIn;
        return *this;
    }

    Stepper& Stepper::setPullOutSpeed(int32_t speed)
    {
        vPullOut = std::abs(speed);
        originalvPullOut = vPullOut;
        return *this;
    }

    Stepper& Stepper::setPullInOutSpeed(int32_t pullInSpeed, int32_t pullOutSpeed)
    {
        setPullInSpeed(pullInSpeed);
        return setPullOutSpeed(pullOutSpeed);
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
        vMax = t.speed * originalvMax;
        setDir(vMax >= 0 ? 1 : -1);
        vPullIn = originalvPullIn + t.vPullIn*(originalvMax - originalvPullIn);
        vPullOut = originalvPullOut + t.vPullOut*(originalvMax - originalvPullOut);
        t.absPos ? setTargetAbs(t.target) : setTargetRel(t.target);
        //Serial.printf("%s loaded target %d, speed %d, pullin: %d, pullout: %d\r\n", name.c_str(), t.target, vMax, vPullIn, vPullOut);
    }

    void Stepper::setTargets(const Target *t, unsigned len)
    {
        if(len == 0) return;
        targets = t;
        targetsLen = len;
        targetsPos = 0;
        // Make sure the first target is loaded for the move call
        loadTarget(targets[targetsPos++]);
    }

    bool Stepper::nextTarget()
    {
        if(targetsPos >= targetsLen)
        {
            A = 0; // set number of steps to zero to flag that there is no more work
                   // to do for this stepper
            return false;
        }
        loadTarget(targets[targetsPos++]);
        return true;
    }
}
