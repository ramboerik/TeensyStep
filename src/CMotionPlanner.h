#ifndef CMOTIONPLANNER_H
#define CMOTIONPLANNER_H

#include <vector>
#include "TeensyStep.h"
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
        void setTarget(int t_x, int t_y, float angle, int &pullin_x, int &pullin_y);

    public:
        CMotionPlanner(Stepper &x, Stepper &y, Stepper &z, int speed, int acc);
        ~CMotionPlanner();
        void addPoint(int x, int y, int z);
        void calculate();
};

#endif
