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

            ObstacleChecker(const ob::SpaceInformationPtr &si, int8_t *obstacle_map, uint16_t width, uint16_t height); 
            /*
             * @brief judge the state whether is valid
             */
            bool isValid(const ob::State *state) const override;

            /*
             * @brief calculate the clearance between states
             */
            double clearance(const ob::State *state) const override;
        private:
            int8_t *_obstacle_grid_map;

            uint16_t _map_width;
            uint16_t _map_height;

            uint16_t _map_width_half;
            uint16_t _map_height_half;

    };
}

#endif
