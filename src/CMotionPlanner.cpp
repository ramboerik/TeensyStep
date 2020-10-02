#include "CMotionPlanner.h"
#include <math.h>

CMotionPlanner::CMotionPlanner(Stepper &x, Stepper &y, Stepper &z, int speed, int acc) : x(x), y(y), z(z), speed(speed), acc(acc){
    x.setAcceleration(acc);
    y.setAcceleration(acc);
    z.setAcceleration(acc);
}

CMotionPlanner::~CMotionPlanner(){
    for(auto it = targets.begin(); it != targets.end();){
        it = targets.erase(it);
    }
}
/**
 * \brief Add point to motion.
 */
void CMotionPlanner::addPoint(int x, int y, int z){
    targets.push_back(new CPoint(x, y, z));
}

/**
 * \brief Calculate angle between two vectors.
 * \return Angle in degrees
 */
float CMotionPlanner::getAngle(int x0, int y0, int x1, int y1){
    float v0_d = sqrtf(x0*x0 + y0*y0);
    float v1_d = sqrtf(x1*x1 + y1*y1);
    float dot = x0*x1 + y0*y1;
    float angle = acosf(dot / (v0_d * v1_d))*180/PI;
    return angle;
}

/**
 * \brief Internal function to set target for the steppers with start/end speed depending
 *        on angle change in motion. If the angle of the motion is changed with more than 45
 *         degrees a full stop/start is done between the targets. If the angle is less than 45
 *         degrees full speed is kept.
 */
void CMotionPlanner::setTarget(int t_x, int t_y, float angle, int &pullin_x, int &pullin_y){
    //Serial.printf("Target (x: %d, y: %d), speed_x: %d, speed_y: %d, angle: ", t_x, t_y, speed, speed);
    //Serial.println(angle);
    if(angle > 45){
        // the change in motion angle is low, we can keep full speed
        x.addTargetAbs(t_x, speed, pullin_x, default_pullin);
        y.addTargetAbs(t_y, speed, pullin_y, default_pullin);
        pullin_x = pullin_y = default_pullin;
    }
    else{
        // the change in motion angle is too large, we need to slow down
        x.addTargetAbs(t_x, speed, pullin_x, speed);
        y.addTargetAbs(t_y, speed, pullin_y, speed);
        pullin_x = speed; // ask stepper for real ve instead of assupmtion that everything is fine?
        pullin_y = speed; // ask stepper for real ve instead of assupmtion that everything is fine?
    }
}
/**
 * \brief Calculate motion, must be done before call to move of steppers. The motion should be re-calculated on
 *        every new call to move, even if no points were removed/added.
 *        NOTE:
 *        * There is currently no way to find out if the steppers managed to reach their end speed(ve) or not
 *          so the assumption is that they always reach it. This can cause undesired high acc/dec if the
 *          end speed wasn't reached for one segment.
 *          Example:
 *          pullin speed = 100
 *          Desired end speed, Ve = 500
 *          start position: (0,0)
 *          targets: 1,1), (10,10)
 *          => Ve won't be reached when moving from (0,0) -> (1,1) but is set as start speed(vs) when starting with movement from (1,1) ->
 *          (10,10). This causes the steppers to accelerate from pullin speed to 500 in one step.
 *
 *        TODO:
 *        * Fix stepper one step bug when nothing to do.
 *        * Speed is given in x and y compntent, not actual speed.
 */
void CMotionPlanner::calculate(){
    int next_pullin_x = default_pullin, next_pullin_y = default_pullin;
    if(targets.empty()){
        return;
    }

    // First point, we must take the current stepper position into consideration.
    float angle = getAngle(x.getPosition(), y.getPosition(), targets[0]->x - x.getPosition(), targets[0]->y - y.getPosition());
    setTarget(targets[0]->x, targets[0]->y, angle, next_pullin_x, next_pullin_y);

    // quit if there's only one target
    if(targets.size() < 2){
        return;
    }

    int x0 = targets[1]->x - targets[0]->x;
    int y0 = targets[1]->y - targets[0]->y;
    unsigned i;
    for(i = 1; i < targets.size() - 1; i++){
        int x1 = targets[i+1]->x - targets[i]->x;
        int y1 = targets[i+1]->y - targets[i]->y;

        angle = getAngle(x0, y0, x1, y1);
        setTarget(targets[i]->x, targets[i]->y, angle, next_pullin_x, next_pullin_y);
        x0 = x1;
        y0 = y1;
    }
    // Add last point, always slow down
    setTarget(targets[i]->x, targets[i]->y, 90, next_pullin_x, next_pullin_y);
}
