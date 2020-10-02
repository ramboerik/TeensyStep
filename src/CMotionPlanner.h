#ifndef CMOTIONPLANNER_H
#define CMOTIONPLANNER_H

#include <vector>
#include "Stepper.h"
#include "CPoint.h"

/**
 *
 * Note: Do not use will pullin/pullout 0 as it might stop the motion before it's done!
 */
class CMotionPlanner{
    private:
        Stepper& x;
        Stepper& y;
        Stepper& z;
        std::vector<CPoint*> targets;
        int speed;
        int acc;

        static constexpr int default_pullin = 100;

        float getAngle(int x0, int y0, int x1, int y1);
        void getSpeed(int delta_x, int delta_y, int &speed_x, int &speed_y);
        void setTarget(int t_x, int t_y, float angle, int speed_x, int speed_y, int &pullin_x, int &pullin_y);

    public:
        // Actual speed will be in the interval [speed, sqrt(2)*speed] as the speed in
        // separated in x and y component.
        // lowest speed is reached when only one axis run.
        // max speed is reached when both axis run equal length.
        CMotionPlanner(Stepper &x, Stepper &y, Stepper &z, int speed, int acc);
        ~CMotionPlanner();
        void addPoint(int x, int y, int z);
        void removePoints();
        void setSpeed(int s) { speed = s; }
        void setAcceleration(int a) {acc = a; }
        void calculate();
};

#endif
