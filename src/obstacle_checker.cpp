#include "../include/obstacle_checker.h"
#include <ompl/base/State.h>

bool RRT_planner::ObstacleChecker::isValid(const ob::State *state) const
{
    return true;
}

double RRT_planner::ObstacleChecker::clearance(const ob::State *state) const
{
    return 0;
}
