#include "../include/obstacle_checker.h"
#include <ompl/base/State.h>
#include <fftw3.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/ros.h>

RRT_planner::ObstacleChecker::ObstacleChecker(const ob::SpaceInformationPtr &si, int8_t *obstacle_map, uint16_t width, uint16_t height) : ob::StateValidityChecker(si)
{
    _map_width  = width;
    _map_height = height;

    _map_width_half  = _map_width / 2;
    _map_height_half = _map_height / 2;

    _obstacle_grid_map = (int8_t*)fftw_malloc(sizeof(int8_t) * _map_width * _map_height);

    for (uint16_t j = 0; j < _map_height; j++)
    {
        for (uint16_t i = 0; i < _map_width; i++)
        {
            _obstacle_grid_map[i + j * _map_width] = obstacle_map[i + j * _map_width]; 
        }
    }
}

bool RRT_planner::ObstacleChecker::isValid(const ob::State *state) const
{
    auto *stateE2 = state->as<ob::SE2StateSpace::StateType>();
    int16_t x_index = static_cast<int16_t>(stateE2->getX()) ;
    int16_t y_index = static_cast<int16_t>(stateE2->getY()) ;

    x_index = stateE2->getX() < static_cast<double>(x_index) ? x_index - 1 : x_index;
    y_index = stateE2->getY() < static_cast<double>(y_index) ? y_index - 1 : y_index;

    x_index += _map_width_half;
    y_index += _map_height_half;

    x_index = x_index < 0 ? 0 : x_index > (_map_width  - 1) ? _map_width  -1 : x_index;
    y_index = y_index < 0 ? 0 : y_index > (_map_height - 1) ? _map_height -1 : y_index;

    //ROS_INFO("the index:%d %d", x_index, y_index);
    if(_obstacle_grid_map[x_index + y_index * _map_width] > 50)
    {
        return false;
    }
    else
    {
        return true;
    }
}

double RRT_planner::ObstacleChecker::clearance(const ob::State *state) const
{
    return 0;
}
