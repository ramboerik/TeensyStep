#pragma once

namespace TeensyStep
{
    class Target
    {
    public:
        Target(int32_t target = 0, float speed = 1, float vPullIn = 0, float vPullOut = 0, bool absPos = true) :
            target(target), speed(speed), vPullIn(vPullIn), vPullOut(vPullOut), absPos(absPos) {}
        int32_t target;
        float speed;
        float vPullIn;
        float vPullOut;
        bool absPos;

        /**
         * \brief Set scale factors for target speed, pullin speed and pullout speed.
         *        speed:
         *              0.0 = zero speed
         *              1.0 = vMax
         *        vPullIn:
         *                0.0: vPullIn
         *                1.0: vMax
         *        vPullOut:
         *                0.0: vPullOut
         *                1.0: vMax
         */
        void setSpeedFactors(float speed, float vPullIn, float vPullOut){
            this->speed = std::min(1.f, speed);
            this->vPullIn = std::min(1.f, vPullIn);
            this->vPullOut = std::min(1.f, vPullOut);
        }
    };
}
