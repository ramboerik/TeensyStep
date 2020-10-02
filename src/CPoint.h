#pragma once

namespace TeensyStep
{
    class CPoint{
        public:
            CPoint(int x, int y, int z): x(x), y(y), z(z) {}
            int x = 0;
            int y = 0;
            int z = 0;
    };
}
