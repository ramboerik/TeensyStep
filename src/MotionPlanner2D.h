#pragma once

#include "Stepper.h"
#include "accelerators/LinStepAccelerator.h"
#include "Target.h"

namespace TeensyStep
{
    /**
     * \brief Implementation of a motion planner with N points buffer
     */
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

            // Reserve two points in the array, one for the start point and
            // one for the last point.
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
                // reserve one place for the end point that is
                // assumed to exist for the algorithm to work.
                if(numTargets >= maxSize -1) return false;

                targetsX[numTargets].target = x;
                targetsY[numTargets].target = y;
                targetsZ[numTargets].target = z;
                numTargets++;
                return true;
            }

            /**
             * \brief Update segments between start(included) and end(excluded) in targets with pullin and pullout speed for each segment.
             */
            void updateSubSegments(Target* targets, unsigned start, unsigned end, int vPullOut, LinStepAccelerator& accelerator)
            {
                for(unsigned i = start; i < end; i++)
                {
                    targets[i].vPullIn  = accelerator.updateSpeed(targets[i - 1].target);
                    targets[i].vPullOut = i == end -1 ? vPullOut : accelerator.updateSpeed(targets[i].target); // updateSpeed returns zero to indicate end of movement, must
                                                                                                               // adjust it to the steppers pullout speed
                    //Serial.printf("Subsegment %d -> %d start speed: %d, end speed: %d\r\n", targets[i - 1].target, targets[i].target, targets[i].vPullIn, targets[i].vPullOut);
                }
            }

            /**
             * \brief Calculate the stepper motion for given targets.
             *
             *  Target/Point: Absolute target for the stepper to visit.
             *  Segment: Collection of continuous targets/points with the same direction.
             *  Motion: The complete movement(acc/constant/dec phase) over a segment.
             *
             *  The algorithm:
             *  1. Iterate all points and determine the steppers direction to each point
             *  2. Divide the motion into segments where each segment is a collection of continuous points with the same direction.
             *  3. Calculate the motion over the segments with LinStepAccelerator.
             *  4. Split the calculated motion over the points in the segment.
             *  5. Repeat until all segments are calculated.
             */
            void updateSegments(Target* targets, unsigned len, int vMax, int pullIn, int pullOut, int acc)
            {
                if(len <= 1)
                {
                    return;
                }
                // Iterate all points and determine the steppers direction to each point
                // targets[0] is always the current stepper position, direction is set to NONE
                // targets[len] is always the last point, direction is set to NONE
                for(unsigned i = 0; i <= len; i++)
                {
                    targets[i].dir = (i == 0 || i == len) ? Target::Direction::NONE : Target::getDirection(targets[i], targets[i - 1]);
                }

                /*
                for(unsigned i = 0; i <= numTargets; i++)
                {
                    Serial.printf("Index: %d: target: %d, direction: %s\r\n", i, targets[i].target, Target::DirectionToStr(targets[i].dir));
                }
                */
                Target::Direction segDirection = Target::Direction::NONE;
                Target::Direction curDirection = Target::Direction::NONE;
                int segStart = -1, segEnd = 0;
                for(unsigned i = 0; i < len; i++)
                {
                    // iterate all targets and look forward for the next direction change.
                    // if a direction change is detected the current segment is calculated and
                    // we start to look for the next segment. Segments with NONE direction are filtered
                    // out. A segment with NONE direction may be start/end position or same position
                    // multiple times in a row.
                    curDirection = targets[i + 1].dir;
                    if(segStart == -1 && curDirection == Target::Direction::NONE)
                    {
                        //Serial.printf("Index: %d, target: %d, Skipping NONE segment\r\n", i, targets[i].target);
                        continue;
                    }

                    if(segStart == -1)
                    {
                        segStart = i;
                        segDirection = curDirection;
                        //Serial.printf("Found segment start at index: %d, direction: %s\r\n", segStart, Target::DirectionToStr(curDirection));
                        continue;
                    }

                    if(segDirection == curDirection)
                    {
                        //Serial.printf("Index: %d, target: %d is subsegment\r\n", i, targets[i].target);
                        continue;
                    }
                    segEnd = i;

                    Serial.printf("Calculating max speed for segment: %d -> %d with direction: %s, (segstart: %d, segend: %d)\r\n",
                                  targets[segStart].target,
                                  targets[segEnd].target,
                                  Target::DirectionToStr(targets[segEnd].dir),
                                  segStart,
                                  segEnd);

                     // calculate the motion for the segment
                    accelerator.prepareMovement(targets[segStart].target, targets[segEnd].target, vMax, pullIn, pullOut, acc);
                    // split the motion over all points in the segment
                    updateSubSegments(targets, segStart + 1, segEnd + 1, pullOut, accelerator);
                    segStart = -1; // start scan of next segment start
                    i--; //current segment's end will be next segments start
                }
            }

            /**
             * \brief Calculate motion, must be done before call to move of steppers.
             */
            void calculate(Stepper &x, Stepper &y)
            {
                // Add start point
                targetsX[0].target = x.getPosition();
                targetsY[0].target = y.getPosition();

                updateSegments(targetsX, numTargets, x.getMaxSpeed(), x.getMaxPullInSpeed(), x.getMaxPullOutSpeed(), x.getAcceleration());
                updateSegments(targetsY, numTargets, y.getMaxSpeed(), y.getMaxPullInSpeed(), y.getMaxPullOutSpeed(), y.getAcceleration());

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
