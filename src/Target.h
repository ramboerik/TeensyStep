#pragma once

namespace TeensyStep
{
    class Target
    {
    public:
        enum Direction
        {
            NONE = 0,
            FORWARD = 1,
            REVERSE = 2
        };

        int32_t target;
        int32_t vPullIn;
        int32_t vPullOut;
        Direction dir;

        Target(int32_t target = 0, int32_t vPullIn = 0, int32_t vPullOut = 0, Direction dir = NONE) :
            target(target), vPullIn(vPullIn), vPullOut(vPullOut), dir(dir) {}

        static Direction getDirection(const Target &start, const Target &end)
        {
            int distance = end.target - start.target;
            if(distance == 0)
            {
                return NONE;
            }
            return distance > 0 ? FORWARD : REVERSE;
        }

        static const char* DirectionToStr(Direction dir)
        {
            switch(dir)
            {
                case FORWARD:
                    return "forward";
                case REVERSE:
                    return "reverse";
                default:
                    return "none";
            }
        }
    };
}
