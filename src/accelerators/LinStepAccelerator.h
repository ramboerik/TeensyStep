#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

class LinStepAccelerator
{
public:
    inline int32_t prepareMovement(int32_t currentPos, int32_t targetPos, uint32_t targetSpeed, uint32_t pullInSpeed, uint32_t pullOutSpeed, uint32_t a);
    inline int32_t updateSpeed(int32_t currentPosition);
    inline uint32_t initiateStopping(int32_t currentPosition);
    inline void overrideSpeed(float fac, int32_t currentPosition);

    LinStepAccelerator() = default;

protected:
    LinStepAccelerator(const LinStepAccelerator &) = delete;
    LinStepAccelerator &operator=(const LinStepAccelerator &) = delete;

    int32_t s_0, ds;
    uint32_t vs, ve, vt;
    int64_t vs_sqr, ve_sqr, vt_sqr;
    uint32_t two_a;
    int32_t accEnd, decStart;
    //int num_acc = 0, num_constant = 0, num_dec = 0;
};

// Inline Implementation =====================================================================================================

int32_t LinStepAccelerator::prepareMovement(int32_t currentPos, int32_t targetPos, uint32_t targetSpeed, uint32_t pullInSpeed, uint32_t pullOutSpeed, uint32_t a)
{
    vt = targetSpeed;
    vs = pullInSpeed;  // v_start
    ve = pullOutSpeed; // v_end
    two_a = 2 * a;

    s_0 = currentPos;
    ds = std::abs(targetPos - currentPos);

    vs_sqr = vs * vs;
    ve_sqr = ve * ve;
    vt_sqr = vt * vt;

    int32_t sa = (std::abs(vt_sqr - vs_sqr)) / two_a; // required distance to reach target speed, starting with start speed
    int32_t se = (std::abs(ve_sqr - vt_sqr)) / two_a; // required distance to reach end speed, starting with target speed
/*
    Serial.printf("ve: %d\r\n", ve);
    Serial.printf("vs: %d\r\n", vs);
    Serial.printf("vt: %d\r\n", vt);
    Serial.printf("ds: %d\r\n", ds);
    Serial.printf("sa: %i\r\n", sa);
    Serial.printf("se: %i\r\n", se);
*/
    if(sa + se > ds) {
        // target speed cannot be reached, need to calculate new max speed and intersecting point between acc and dec curve
        vt = sqrtf((ve_sqr - vs_sqr) / 2); // speed at acc/dec curve intersect point
        vt_sqr = vt * vt;
        sa = (std::abs(vt_sqr - vs_sqr)) / two_a; // acc distance
        se = (std::abs(ve_sqr - vt_sqr)) / two_a; // dec distance
        //Serial.printf("Recalculating, new max speed is: %d\r\n", vt);
    }
    accEnd = sa;
    decStart = ds - se;
    //Serial.printf("acc end: %d, dec start: %d, total: %d\r\n", accEnd, decStart, ds);
    //Serial.printf("stats: num_acc: %d, num_constant: %d, num_dec: %d\r\n", num_acc, num_constant, num_dec);
    //num_acc = num_constant = num_dec = 0;
    return vs;
}

int32_t LinStepAccelerator::updateSpeed(int32_t curPos)
{

    int32_t s = std::abs(s_0 - curPos);

    // acceleration phase -------------------------------------
    if (s < accEnd)
    {
        //num_acc++;
        return vs + (vt > vs ? sqrtf(two_a * s) : -sqrtf(two_a * s)); // handle both acc and dec
    }

    // constant speed phase ------------------------------------
    if (s < decStart)
    {
        //num_constant++;
        return vt;
    }

    //deceleration phase --------------------------------------
    if (s < ds)
    {
        //num_dec++;
        int32_t dec_s = s - decStart;
        return vt + (ve > vt ? sqrtf(dec_s * two_a) : -sqrtf(dec_s * two_a)); // handle both acc and dec
    }

    //we are done, make sure to return 0 to stop the step timer
    return 0;
}

uint32_t LinStepAccelerator::initiateStopping(int32_t curPos)
{
    int32_t stepsDone = std::abs(s_0 - curPos);

    if (stepsDone < accEnd) // still accelerating
    {
        accEnd = decStart = 0; // start deceleration
        ds = 2 * stepsDone;    // we need the same way to decelerate as we traveled so far
        return stepsDone;      // return steps to go
    }
    else if (stepsDone < decStart) // constant speed phase
    {
        decStart = 0;            // start deceleration
        ds = stepsDone + accEnd; // normal deceleration distance
        return accEnd;           // return steps to go
    }
    else // already decelerating
    {
        return ds - stepsDone; // return steps to go
    }
}
