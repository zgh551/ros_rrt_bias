#ifndef _OBSTACLE_CHECKER_H_
#define _OBSTACLE_CHECKER_H_

#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

namespace ob = ompl::base;

namespace RRT_planner
{
    class ObstacleChecker : public ob::StateValidityChecker 
    {
        public:
            ObstacleChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
            {

            }
            
            /*
             * @brief judge the state whether is valid
             */
            bool isValid(const ob::State *state) const override;

            /*
             * @brief calculate the clearance between states
             */
            double clearance(const ob::State *state) const override;

    };
}

#endif
