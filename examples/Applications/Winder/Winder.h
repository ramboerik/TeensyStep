#pragma once

#include "TeensyStep.h"

class Winder
{
public:
    Winder(Stepper &spindle, Stepper &feeder);

    void begin();

    Winder &setSpindleParams(unsigned stpPerRev, unsigned acceleration); // steps per spindle revolution & maximum spindle speed in rpm
    Winder &setFeederParams(unsigned stpPerMM);                                        // steps to move the feeder 1mm

    void setSpindleSpeed(float rpm);  // changes the spindle speed to the given rpm
    void setPitch(float pitch_in_mm); // changes the winder pitch to the given value
    void updateSpeeds();

    inline int getCurSpindleSpeed() { return spindleCtrl.isRunning() ? spindleCtrl.getCurrentSpeed() : 0; }
    inline int getCurFeederSpeed() { return feederCtrl.isRunning() ? feederCtrl.getCurrentSpeed() : 0; }
    inline float getCurPitch(){return (float)getCurFeederSpeed()/getCurSpindleSpeed()/pitchFactor; }

protected:
    Stepper &spindle;
    Stepper &feeder;

    unsigned spindleStpPerRev, feederStpPerMM;
    float pitchFactor;

    float targetSpindleSpeed, targetPitch;
    float oldSpindleSpeed, oldPitch;

    RotateControl feederCtrl, spindleCtrl;
};