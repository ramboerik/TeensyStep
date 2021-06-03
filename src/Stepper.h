#pragma once

#include <algorithm>
#include <cstdint>
#include "Target.h"

namespace TeensyStep
{
    class Stepper
    {

     public:
        static const int32_t vMaxMax;
        static const uint32_t aMax;
        static const uint32_t vMaxDefault;
        static const uint32_t vPullInOutDefault;
        static const uint32_t aDefault;

        Stepper(const int StepPin, const int DirPin, const char* name = "");

        virtual Stepper& setMaxSpeed(int32_t speed);                          // steps/s
        Stepper& setPullInSpeed(int32_t speed);                               // steps/s
        Stepper& setPullOutSpeed(int32_t speed);                               // steps/s
        virtual Stepper& setPullInOutSpeed(int32_t pullInpeed, int32_t pullOutSpeed); // steps/s
        virtual Stepper& setAcceleration(uint32_t _a);                        // steps/s^2

        Stepper& setStepPinPolarity(int p);  // HIGH -> positive pulses, LOW -> negative pulses
        Stepper& setInverseRotation(bool b); // Change polarity of the dir pulse

        virtual void setTargetAbs(int32_t pos);   // Set target position absolute
        virtual void setTargetRel(int32_t delta); // Set target position relative to current position
        void setTargets(const Target *targets, unsigned len);
        bool nextTarget();
        const char* getName() { return name; }

        int32_t getMaxSpeed() { return vMax; }
        int32_t getAcceleration() { return a; }
        int32_t getMaxPullInSpeed() { return vPullInMax; }
        int32_t getMaxPullOutSpeed() { return vPullOutMax; }


        inline int32_t getPosition() const { return current; }
        inline void setPosition(int32_t pos) { current = pos; }
        int32_t dir;

     protected:
        void loadTarget(const Target& t);
        void reset();

        inline void doStep();
        inline void clearStepPin() const;

        virtual void setDir(int d);
        void toggleDir();

        // positions
        volatile int32_t current;
        volatile int32_t currentSpeed;
        volatile int32_t target;
        const Target* targets;
        unsigned targetsLen = 0;
        unsigned targetsPos = 0;

        int32_t A, B; // Bresenham paramters
        int32_t vMax;
        int32_t vPullIn, vPullOut;
        int32_t vPullInMax, vPullOutMax; // store configured pullin/pullout speeds
                                         // in separate variables as they get overwritten
                                         // when loading motion targets
        uint32_t a;

        // compare functions
        static bool cmpDelta(const Stepper* a, const Stepper* b) { return a->A > b->A; }
        static bool cmpAcc(const Stepper* a, const Stepper* b) { return a->a < b->a; }
        static bool cmpVmin(const Stepper* a, const Stepper* b) { return std::abs(a->vMax) < std::abs(b->vMax); }
        static bool cmpVmax(const Stepper* a, const Stepper* b) { return std::abs(a->vMax) > std::abs(b->vMax); }
        static bool cmpPullIn(const Stepper* a, const Stepper* b) { return a->vPullIn > b->vPullIn; }
        static bool cmpPullOut(const Stepper* a, const Stepper* b) { return a->vPullOut > b->vPullOut; }

        // Pin & Dir registers
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        volatile uint32_t* stepPinActiveReg;
        volatile uint32_t* stepPinInactiveReg;
        volatile uint32_t* dirPinCwReg;
        volatile uint32_t* dirPinCcwReg;
#else
        volatile uint8_t polarity;
        volatile uint8_t reverse;
#endif
        const int stepPin, dirPin;
        char name[32];

        // Friends
        template <typename a, typename t>
        friend class StepControlBase;

        template <typename a, typename t>
        friend class RotateControlBase;

        template <typename t>
        friend class MotorControlBase;
    };
    // Inline implementation -----------------------------------------
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    void Stepper::doStep()
    {
        *stepPinActiveReg = 1;
        current += dir;
    }

    void Stepper::clearStepPin() const
    {
        *stepPinInactiveReg = 1;
    }

#else
    void Stepper::doStep()
    {
        digitalWrite(stepPin, polarity);
        current += dir;
    }

    void Stepper::clearStepPin() const
    {
        digitalWrite(stepPin, !polarity);
    }
#endif
}
