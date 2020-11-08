#pragma once

#include "Stepper.h"
#include "accelerators/LinStepAccelerator.h"
#include "Target.h"

namespace TeensyStep
{
    template<unsigned N = 100>
    class MotionPlanner2D
    {
        protected:
            static constexpr unsigned maxSize = N;
            unsigned numTargets = 1;
            Target targetsX[maxSize];
            Target targetsY[maxSize];
            Target targetsZ[maxSize];
            LinStepAccelerator accelerator;

        public:
            MotionPlanner2D() {}
            // Actual speed will be in the interval [speed, sqrt(2)*speed] as the speed in
            // separated in x and y component.
            // lowest speed is reached when only one axis run.
            // max speed is reached when both axis run equal length.

            // target zero is always the steppers current position
            unsigned size() { return numTargets - 1; }
            void clear() { numTargets = 1; }
            Target* getX() { return &targetsX[1]; }
            Target* getY() { return &targetsY[1]; }
            Target* getZ() { return &targetsZ[1]; }

            /**
             * \brief Add point to motion.
             */
            bool addPoint(int x, int y, int z = 0)
            {
                if(numTargets >= maxSize) return false;
                targetsX[numTargets].target = x;
                targetsY[numTargets].target = y;
                targetsZ[numTargets].target = z;
                numTargets++;
                return true;
            }

            /**
             * \brief Update all segments between start and end in targets with pullin and pullout speed for each segment
             */
            void updateSubSegments(Target* targets, unsigned start, unsigned end, int vPullOut, LinStepAccelerator& accelerator)
            {
                for(unsigned i = start + 1; i <= end; i++)
                {
                    targets[i].vPullIn  = accelerator.updateSpeed(targets[i - 1].target);
                    targets[i].vPullOut = i == end ? vPullOut : accelerator.updateSpeed(targets[i].target); // updateSpeed returns zero to indicate end of movement, must
                                                                                                            // adjust it to the steppers pullout speed

                    Serial.printf("Subsegment %d -> %d start speed: %d, end speed: %d\r\n", targets[i - 1].target, targets[i].target, targets[i].vPullIn, targets[i].vPullOut);
                }
            }

            /**
             * \brief
             *  A segment is defiend as continious targets that don't change the steppers direction. A change in direction defines the start of a new segment.
             */
            void updateSegments(Target* targets, unsigned len, int vMax, int pullIn, int pullOut, int acc)
            {
                if(len <= 1)
                {
                    return;
                }

                // iterate whole motion and divide it into segments
                // e.g a path that is 10, 20, 30, 40 will be flagged as the same part
                //                    10, 20, 10, 0 wiull be two segments: 10, 20 and 10, 0 as the direction changes
                targets[0].dir = Target::Direction::NONE;
                for(unsigned i = 1; i < len; i++)
                {
                    targets[i].dir = Target::getDirection(targets[i], targets[i-1]);
                }

                for(unsigned i = 0; i < numTargets; i++)
                {
                    Serial.printf("Index: %d: target: %d, direction: %s\r\n", i, targets[i].target, Target::DirectionToStr(targets[i].dir));
                }

                // calculate max speed for all segments
                Target::Direction segDirection = Target::Direction::NONE;
                Target::Direction curDirection = Target::Direction::NONE;
                int segStart = -1, segEnd = 0;

                for(unsigned i = 1; i <= len; i++)
                {
                    curDirection = i < len ? targets[i].dir : Target::Direction::NONE;
                    // skip segment with no motion(aka direction NONE)
                    if(segStart == -1 && curDirection == Target::Direction::NONE)
                    {
                        Serial.printf("Index: %d, target: %d, Skipping NONE segment\r\n", i, targets[i].target);
                        continue;
                    }

                    if(segStart == -1)
                    {
                        segStart = i - 1;
                        segDirection = curDirection;
                        Serial.printf("Found segment start at index: %d, direction: %s\r\n", segStart, Target::DirectionToStr(curDirection));
                        continue;
                    }

                    if(segDirection == curDirection)
                    {
                        Serial.printf("Index: %d, target: %d is subsegment\r\n", i, targets[i].target);
                        continue;
                    }
                    segEnd = i - 1;

                    Serial.printf("Calculating max speed for segment: %d -> %d with direction: %s, (segstart: %d, segend: %d)\r\n",
                                  targets[segStart].target,
                                  targets[segEnd].target,
                                  Target::DirectionToStr(targets[segEnd].dir),
                                  segStart,
                                  segEnd);

                     // calculate the maximum speed for the segment
                    accelerator.prepareMovement(targets[segStart].target, targets[segEnd].target, vMax, pullIn, pullOut, acc);
                    updateSubSegments(targets, segStart, segEnd, pullOut, accelerator);
                    segStart = -1;
                    i--; //current segments end will be next segments start
                }
            }

            /**
             * \brief Calculate motion, must be done before call to move of steppers.
             */
            void calculate(Stepper& x, Stepper&y)
            {
                targetsX[0].target = x.getPosition();
                targetsY[0].target = y.getPosition();

                //updateSegments(targetsX, numTargets, x.vMax, x.vPullIn, x.vPullOut, x.aMax);
                updateSegments(targetsX, numTargets, 500, 100, 100, 2000);
                updateSegments(targetsY, numTargets, 500, 100, 100, 2000);
                //updateSegments(targetsY, numTargets, y.vMax, y.vPullIn, y.vPullOut, y.aMax);
                x.setTargets(getX(), size());
                y.setTargets(getY(), size());

                Serial.println("==== DUMP X====");
                for(unsigned i = 0; i < size(); i++)
                {
                    Serial.printf("pos: %d, pullin: %d, pullout: %d\r\n", getX()[i].target, getX()[i].vPullIn, getX()[i].vPullOut);
                }
                Serial.println("==== DUMP Y====");
                for(unsigned i = 0; i < size(); i++)
                {
                    Serial.printf("pos: %d, pullin: %d, pullout: %d\r\n", getY()[i].target, getY()[i].vPullIn, getY()[i].vPullOut);
                }
                Serial.println("========");
            }
    };
}
