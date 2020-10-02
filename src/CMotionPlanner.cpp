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
 * \brief Calculate the x' and y' axis speed that combined gives desired speed given in
 *        constructor.
 *        NOTE: Not supported as there is no way to set individual speeds when moving in sync.
 * \param[in] delta_x X axis delta of target
 * \param[in] delta_y Y axis delta of target
 * \param[out] speed_x Output x axis speed
 * \param[out] speed_y Output y axis speed
 */
void CMotionPlanner::getSpeed(int delta_x, int delta_y, int &speed_x, int &speed_y){
    float len = sqrtf(delta_x*delta_x + delta_y*delta_y);
    speed_x = std::abs(speed*(delta_x/len));
    speed_y = std::abs(speed*(delta_y/len));
}

/**
 * \brief Internal function to set target for the steppers with start/end speed depending
 *        on angle change in motion. If the angle of the motion is changed with more than 45
 *         degrees a full stop/start is done between the targets. If the angle is less than 45
 *         degrees full speed is kept.
 */
void CMotionPlanner::setTarget(int t_x, int t_y, float angle, int speed_x, int speed_y, int &pullin_x, int &pullin_y){
    Serial.printf("Target (x: %d, y: %d), speed_x: %d, speed_y: %d, angle: ", t_x, t_y, speed_x, speed_y);
    Serial.println(angle);

    if(angle < 45){
        // the change in motion angle is too large, we need to slow down
        x.addTargetAbs(t_x, speed, pullin_x, speed_x);
        y.addTargetAbs(t_y, speed, pullin_y, speed_y);
        pullin_x = speed_x; // ask stepper for real ve instead of assuming that everything is fine?
        pullin_y = speed_y; // ask stepper for real ve instead of assuming that everything is fine?
    }
    else{
        // the change in motion angle is low, we can keep full speed
        x.addTargetAbs(t_x, speed_x, pullin_x, default_pullin);
        y.addTargetAbs(t_y, speed_y, pullin_y, default_pullin);
        pullin_x = pullin_y = default_pullin;
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
 *        * Bug, the steppers do one step backwards and forwards when giving same point as target multiple times.
 */
void CMotionPlanner::calculate(){
    int next_pullin_x = default_pullin, next_pullin_y = default_pullin;
    int speed_x = speed, speed_y = speed;
    if(targets.empty()){
        return;
    }

    // First point, we must take the current stepper position into consideration.
    int x0 = targets[0]->x - x.getPosition();
    int y0 = targets[0]->y - y.getPosition();
    float angle = getAngle(x.getPosition(), y.getPosition(), x0, y0);
    //getSpeed(x0, y0, speed_x, speed_y);
    setTarget(targets[0]->x, targets[0]->y, angle, speed_x, speed_y, next_pullin_x, next_pullin_y);

    // quit if there's only one target
    if(targets.size() < 2){
        return;
    }

    x0 = targets[1]->x - targets[0]->x;
    y0 = targets[1]->y - targets[0]->y;
    unsigned i;
    for(i = 1; i < targets.size() - 1; i++){
        int x1 = targets[i+1]->x - targets[i]->x;
        int y1 = targets[i+1]->y - targets[i]->y;

        //angle = getAngle(targets[i]->x, targets[i]->y, x0, y0);
        angle = getAngle(x0, y0, x1, y1);
        //getSpeed(x1, y1, speed_x, speed_y);
        setTarget(targets[i]->x, targets[i]->y, angle, speed_x, speed_y, next_pullin_x, next_pullin_y);
        x0 = x1;
        y0 = y1;
    }
    // Add last point, always slow down
    //getSpeed(targets[i]->x - targets[i - 1]->x, targets[i]->y - targets[i - 1]->y, speed_x, speed_y);
    setTarget(targets[i]->x, targets[i]->y, 90, speed_x, speed_y, next_pullin_x, next_pullin_y);
}
