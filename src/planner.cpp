/*
 * @brief The planner 
 */
#include "../include/planner.h"
#include "../include/rrt_bias.h"
#include "../include/obstacle_checker.h"
#include "../include/convolution.h"

// the tf
#include <algorithm>
#include <fftw3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

/*
 * @brief OMPL lib
 */
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <ompl/geometric/planners/informedtrees/AITstar.h>

/*
 * @brief System lib
 */
#include <valarray>
#include <memory>
#include <vector>
#include <visualization_msgs/Marker.h>

using namespace Common;

RRT_planner::Planner::Planner(void)
{
    _start_valid = false;
    _goal_valid  = false;
    _is_map_update = false;

    _disk_map_width  = 7;
    _disk_map_height = 7;
    _disk_map_size   = _disk_map_width * _disk_map_height;
    _disk_origin_x = static_cast<int16_t>(-(_disk_map_width  - 1) * 0.5);
    _disk_origin_y = static_cast<int16_t>(-(_disk_map_height - 1) * 0.5);

    _obstacle_map_width  = BOUNDARY_SIZE_X;
    _obstacle_map_height = BOUNDARY_SIZE_Y;
    _obstacle_map_size   = _obstacle_map_width * _obstacle_map_height;
    _obstacle_origin_x = static_cast<int16_t>(-_obstacle_map_width * 0.5);
    _obstacle_origin_y = static_cast<int16_t>(-_obstacle_map_height * 0.5);

    /*
     * @brief malloc the memory space for grid map
     */
    _disk_grid_map = (int8_t*)fftw_malloc(sizeof(int8_t) * _disk_map_size);

    /*
     * @brief base on the radius of circle produce the grid map
     */
    DrawCircle(2);

    // the line strip init
    _start_pose_line_strip.header.frame_id = 
    _goal_pose_line_strip.header.frame_id  =
    _sample_point.header.frame_id =
    _plan_path.header.frame_id = 
    _disk_occ_map.header.frame_id = 
    _convolution_occ_map.header.frame_id = 
    "map";


    _start_pose_line_strip.header.stamp = 
    _goal_pose_line_strip.header.stamp  =
    _sample_point.header.stamp = 
    _plan_path.header.stamp = 
    _disk_occ_map.header.stamp = 
    _convolution_occ_map.header.stamp = 
    ros::Time::now();

     _start_pose_line_strip.ns = "start_pose_line_strip";
     _goal_pose_line_strip.ns  = "goal_pose_line_strip";

     _start_pose_line_strip.action = 
     _goal_pose_line_strip.action  =
     visualization_msgs::Marker::MODIFY;

     _start_pose_line_strip.id = 0;
     _goal_pose_line_strip.id  = 1;


     _start_pose_line_strip.type = 
     _goal_pose_line_strip.type  =
     visualization_msgs::Marker::ARROW;

    // the line strip scale 
    _start_pose_line_strip.scale.x = _goal_pose_line_strip.scale.x = 1;
    _start_pose_line_strip.scale.y = _goal_pose_line_strip.scale.y = 0.1;
    _start_pose_line_strip.scale.z = _goal_pose_line_strip.scale.z = 0.1;

    _start_pose_line_strip.color.b = 1.0;
    _start_pose_line_strip.color.a = 1.0;

    _goal_pose_line_strip.color.g = 1.0;
    _goal_pose_line_strip.color.a = 1.0;

    /*
     * @brief The disk map configure
     */
    _disk_occ_map.info.height =  _disk_map_height;
    _disk_occ_map.info.width  =  _disk_map_width;
    _disk_occ_map.info.origin.position.x = _disk_origin_x;
    _disk_occ_map.info.origin.position.y = _disk_origin_y;
    _disk_occ_map.info.resolution = 1;

    _convolution_occ_map.info.resolution = 1;

    /*
     * @brief The publisher
     */
    line_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    pose_array_pub  = n.advertise<geometry_msgs::PoseArray>("sampling_points", 100);
    path_state_pub  = n.advertise<nav_msgs::Path>("plan_path", 100);
    disk_grid_map_pub = n.advertise<nav_msgs::OccupancyGrid>("disk_grid", 100);
    sum_grid_map_pub    = n.advertise<nav_msgs::OccupancyGrid>("sum_grid", 100);

    /*
     * @brief The subscribe
     */
    map_sub        = n.subscribe("map", 1, &RRT_planner::Planner::MapCallback, this);
    start_pose_sub = n.subscribe("initialpose", 1, &RRT_planner::Planner::StartPoseCallback, this);
    goal_pose_sub  = n.subscribe("move_base_simple/goal", 1, &RRT_planner::Planner::GoalPoseCallback, this);

}

RRT_planner::Planner::~Planner(void)
{
    // TODO free memory
}

/*
 * @brief The initial function
 */
void RRT_planner::Planner::Init(void)
{
    ob::RealVectorBounds obstacle_boundary = ob::RealVectorBounds(2);

     
    obstacle_boundary.setLow(0, _obstacle_origin_x);
    obstacle_boundary.setLow(1, _obstacle_origin_y);

    obstacle_boundary.setHigh(0, _obstacle_map_width  + _obstacle_origin_x);
    obstacle_boundary.setHigh(1, _obstacle_map_height + _obstacle_origin_y);

    _state_space = std::make_shared<ob::ReedsSheppStateSpace>(5.0);
    //_state_space = std::make_shared<ob::SE2StateSpace>();
    _state_space->as<ob::SE2StateSpace>()->setBounds(obstacle_boundary);

    _si = std::make_shared<ob::SpaceInformation>(_state_space);
    _si->setStateValidityChecker(std::make_shared<RRT_planner::ObstacleChecker>(_si));
    _si->setStateValidityCheckingResolution(0.03);
    _si->setup();

    _ss = std::make_shared<og::SimpleSetup>(_si);
    _ss->setPlanner(std::make_shared<og::RRT_Bias>(_si));

}

void RRT_planner::Planner::Init(int8_t *map, uint16_t map_width, uint16_t map_height)
{
    ob::RealVectorBounds obstacle_boundary = ob::RealVectorBounds(2);

    obstacle_boundary.setLow(0, -map_width  / 2);
    obstacle_boundary.setLow(1, -map_height / 2);

    obstacle_boundary.setHigh(0, map_width  / 2);
    obstacle_boundary.setHigh(1, map_height / 2);

    _state_space = std::make_shared<ob::ReedsSheppStateSpace>(5.0);
    //_state_space = std::make_shared<ob::SE2StateSpace>();
    _state_space->as<ob::SE2StateSpace>()->setBounds(obstacle_boundary);

    _si = std::make_shared<ob::SpaceInformation>(_state_space);
    _si->setStateValidityChecker(std::make_shared<RRT_planner::ObstacleChecker>(_si, map, map_width, map_height));
    _si->setStateValidityCheckingResolution(0.03);
    _si->setup();

    _ss = std::make_shared<og::SimpleSetup>(_si);
    _ss->setPlanner(std::make_shared<og::RRT_Bias>(_si));
   //_ss->setPlanner(std::make_shared<og::BiTRRT>(_si));

}
/*
 * @brief The solve function
 */
void RRT_planner::Planner::solve(const double time)
{
    if (_start_valid && _goal_valid)
    {
        ob::ScopedState<> start_state(_state_space);
        ob::ScopedState<> goal_state(_state_space);

        start_state->as<ob::SE2StateSpace::StateType>()->setX(_start_position.x);
        start_state->as<ob::SE2StateSpace::StateType>()->setY(_start_position.y);
        start_state->as<ob::SE2StateSpace::StateType>()->setYaw(_start_position.yaw);

        goal_state->as<ob::SE2StateSpace::StateType>()->setX(_goal_position.x);
        goal_state->as<ob::SE2StateSpace::StateType>()->setY(_goal_position.y);
        goal_state->as<ob::SE2StateSpace::StateType>()->setYaw(_goal_position.yaw);

        _ss->setStartAndGoalStates(start_state, goal_state);

        _ss->setup();
        _ss->print();

        ob::PlannerStatus solved = _ss->solve(time);

        if (solved)
        {
            /*
             * @brief Step1: the solution path state points
             */
            _sample_point.poses.clear();
            geometry_msgs::Pose pose_temp;
            for (auto &state : _ss->getSolutionPath().getStates())
            {
                const auto *SE2_state = state->as<ob::SE2StateSpace::StateType>();
                pose_temp.position.x  = SE2_state->getX();
                pose_temp.position.y  = SE2_state->getY();
                pose_temp.orientation = tf::createQuaternionMsgFromYaw(SE2_state->getYaw());
                _sample_point.poses.push_back(pose_temp);
            }
            pose_array_pub.publish(_sample_point);

            /*
             * @brief Step2: base on the solution path points,interpolate the
             * full path with small step lenght
             */
            og::PathGeometric path_state = _ss->getSolutionPath();

            // using the solution path to interpolate the state
            path_state.interpolate(1000);
            geometry_msgs::PoseStamped pose_stamp;
            _plan_path.poses.clear();
            for (auto &state : path_state.getStates())
            {
                auto *SE2_state = state->as<ob::SE2StateSpace::StateType>();
                pose_stamp.pose.position.x = SE2_state->getX();
                pose_stamp.pose.position.y = SE2_state->getY();
                _plan_path.poses.push_back(pose_stamp);
            }
            path_state_pub.publish(_plan_path);
        }
        else 
        {
            
        }
        _ss->clear();
        _goal_valid = false;
    }
}


void RRT_planner::Planner::DrawCircle(double r)
{
    for (uint16_t i = 0; i < _disk_map_size; i++)
    {
        _disk_grid_map[i] = 0;
    }

    int16_t F = 1 - r;
    int16_t x = r;
    int16_t y = 0;
    int16_t delta_up_left = -2 * r;
    int16_t delta_up      = 1;
    while (y < x)
    {
        if (F >= 0)
        {
            for(uint16_t i = -x - _disk_origin_x; i < (x - _disk_origin_x + 1); i++)
            {
                for (uint16_t j = -y -_disk_origin_y; j < (y - _disk_origin_y + 1); j++)
                {
                    _disk_grid_map[j + _disk_map_width * i] = 100;
                }
            }
            for(uint16_t i = -y - _disk_origin_y; i < (y - _disk_origin_y + 1); i++)
            {
                for (uint16_t j = -x - _disk_origin_x; j < (x - _disk_origin_x + 1); j++)
                {
                    _disk_grid_map[j + _disk_map_width * i] = 100;
                }
            }
            x -= 1;
            delta_up_left += 2;
            F += delta_up_left;
        }
        y += 1;
        delta_up += 2;
        F += delta_up;
    }
}


/*
 * @brief the callback function for 
 */
void RRT_planner::Planner::MapCallback(const nav_msgs::OccupancyGrid::Ptr map)
{
    /*
     * @brief obstacle map size update
     */
    _obstacle_map_height = map->info.height;   
    _obstacle_map_width  = map->info.width;
    _obstacle_map_size   = _obstacle_map_width * _obstacle_map_height;
    _obstacle_origin_x   = static_cast<int16_t>(-_obstacle_map_width  * 0.5);
    _obstacle_origin_y   = static_cast<int16_t>(-_obstacle_map_height * 0.5);

    if (_is_map_update)
    {
        fftw_free(_obstacle_grid_map);
        fftw_free(_convolution_grid_map);
    }
    else
    {
        _is_map_update = true;
    }
    _obstacle_grid_map    = (int8_t*)fftw_malloc(sizeof(int8_t) * _obstacle_map_size);
    _convolution_grid_map = (int8_t*)fftw_malloc(sizeof(int8_t) * _obstacle_map_size); 

    _convolution_occ_map.info.height = _obstacle_map_height;
    _convolution_occ_map.info.width  = _obstacle_map_width;
    _convolution_occ_map.info.origin.position.x = _obstacle_origin_x;
    _convolution_occ_map.info.origin.position.y = _obstacle_origin_y;

    for (uint16_t i = 0; i < _obstacle_map_height; i++)
    {
        for (uint16_t j = 0; j < _obstacle_map_width; j++) 
        {
            _obstacle_grid_map[i * _obstacle_map_width + j] = map->data[i * _obstacle_map_width + j] > 0 ? 100 : 0;
        }
    }
    ROS_INFO("grid map update width:%d,height:%d", _obstacle_map_width, _obstacle_map_height);

    ros::Time begin = ros::Time::now();
//    convolution_2d(_disk_grid_map, _obstacle_grid_map, _convolution_grid_map);
    Common::Convolution::convolution_2d( _disk_grid_map, _disk_map_width, _disk_map_height,
                                       _obstacle_grid_map,_obstacle_map_width, _obstacle_map_height, 
                                       _convolution_grid_map);
    
    double pro_time = (ros::Time::now() - begin).toSec();

    ROS_INFO("convolution time %f", pro_time);
    /*
     * @brief the convolution result map show
     */
    std::vector<int8_t> convolution_map_temp(_convolution_grid_map , _convolution_grid_map + _obstacle_map_size);
    _convolution_occ_map.data = convolution_map_temp;
    sum_grid_map_pub.publish(_convolution_occ_map);

    /*
     * @brief the disk map show
     */
    std::vector<int8_t> disk_map_temp(_disk_grid_map, _disk_grid_map + _disk_map_size );
    _disk_occ_map.data = disk_map_temp;
    disk_grid_map_pub.publish(_disk_occ_map);

    /*
     * @brief init the planner space and obstacle map
     */
    Init(_convolution_grid_map, _obstacle_map_width, _obstacle_map_height);
}

/*
 * @brief The callback function of start position
 */
void RRT_planner::Planner::StartPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    _start_position.x = pose.pose.pose.position.x;
    _start_position.y = pose.pose.pose.position.y;
    _start_position.yaw = tf::getYaw(pose.pose.pose.orientation);

    _start_valid = true;

    _start_pose_line_strip.pose = pose.pose.pose;

    line_marker_pub.publish(_start_pose_line_strip);

    ROS_INFO("start position x:%f, y:%f, yaw:%f",   _start_position.x,
                                                    _start_position.y,
                                                    _start_position.yaw);

}

/*
 * @brief The callback function of goal position
 */
void RRT_planner::Planner::GoalPoseCallback(const geometry_msgs::PoseStamped &pose)
{
    _goal_position.x = pose.pose.position.x;
    _goal_position.y = pose.pose.position.y;
    _goal_position.yaw = tf::getYaw(pose.pose.orientation);

    _goal_valid = true;

    _goal_pose_line_strip.pose = pose.pose;

    line_marker_pub.publish(_goal_pose_line_strip);

    ROS_INFO("goal position x:%f, y:%f, yaw:%f", _goal_position.x,
                                                 _goal_position.y,
                                                 _goal_position.yaw);
}


