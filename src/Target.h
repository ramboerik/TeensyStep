#pragma once

namespace TeensyStep
{
    class Target
    {
    public:
        Target(int32_t target = 0, int32_t speed = 0, int32_t vPullIn = -1, int32_t vPullOut = -1, bool absPos = true) :
            target(target), speed(speed), vPullIn(vPullIn), vPullOut(vPullOut), absPos(absPos) {}
        int32_t target;
        int32_t speed;
        int32_t vPullIn;
        int32_t vPullOut;
        bool absPos;

        void setSpeeds(int speed, int vPullIn, int vPullOut){
            this->speed = speed;
            this->vPullIn = vPullIn;
            this->vPullOut = vPullOut;
        }
    };
}
