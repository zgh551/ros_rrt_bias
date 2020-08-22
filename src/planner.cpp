/*
 * @brief The planner 
 */
#include "../include/planner.h"
#include "../include/rrt_bias.h"
#include "../include/obstacle_checker.h"

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

/*
 * @brief System lib
 */
#include <valarray>
#include <memory>
#include <vector>
#include <visualization_msgs/Marker.h>


RRT_planner::Planner::Planner(void)
{
    _start_valid = false;
    _goal_valid  = false;

    _map_width  = BOUNDARY_SIZE_X;
    _map_height = BOUNDARY_SIZE_Y;
    _map_size   = BOUNDARY_SIZE_X * BOUNDARY_SIZE_Y;
    _map_sum_size =  _map_size ;

    _origin_x = static_cast<int16_t>(-BOUNDARY_SIZE_X * 0.5);
    _origin_y = static_cast<int16_t>(-BOUNDARY_SIZE_Y * 0.5);
    // the line strip init
    _start_pose_line_strip.header.frame_id = 
    _goal_pose_line_strip.header.frame_id  =
    _sample_point.header.frame_id =
    _plan_path.header.frame_id = 
    _disk_occ_map.header.frame_id = 
    _sum_occ_map.header.frame_id = 
    "map";


    _start_pose_line_strip.header.stamp = 
    _goal_pose_line_strip.header.stamp  =
    _sample_point.header.stamp = 
    _plan_path.header.stamp = 
    _disk_occ_map.header.stamp = 
    _sum_occ_map.header.stamp = 
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
    _disk_occ_map.info.height =  _map_height;
    _disk_occ_map.info.width  =  _map_width;
    _disk_occ_map.info.origin.position.x = _origin_x;
    _disk_occ_map.info.origin.position.y = _origin_y;
    _disk_occ_map.info.resolution = 1;

    _sum_occ_map.info.height = _map_height;
    _sum_occ_map.info.width  = _map_width;
    _sum_occ_map.info.origin.position.x = _origin_x;
    _sum_occ_map.info.origin.position.y = _origin_y;
    _sum_occ_map.info.resolution = 1;

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

     
    obstacle_boundary.setLow(0, _origin_x);
    obstacle_boundary.setLow(1, _origin_y);

    obstacle_boundary.setHigh(0, _map_width  + _origin_x);
    obstacle_boundary.setHigh(1, _map_height + _origin_y);

    //_state_space = std::make_shared<ob::ReedsSheppStateSpace>(5.0);
    _state_space = std::make_shared<ob::SE2StateSpace>();
    _state_space->as<ob::SE2StateSpace>()->setBounds(obstacle_boundary);

    _si = std::make_shared<ob::SpaceInformation>(_state_space);
    _si->setStateValidityChecker(std::make_shared<RRT_planner::ObstacleChecker>(_si));
    _si->setStateValidityCheckingResolution(0.03);
    _si->setup();

    _ss = std::make_shared<og::SimpleSetup>(_si);
    _ss->setPlanner(std::make_shared<og::RRT_Bias>(_si));

    _disk_array             = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);
    _obstacle_array         = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);
    _c_ifft_input_sum_array = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);
    _ifft_output_sum_array  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);

//    DrawCircle(2);
//    for (uint16_t j = 0; j < 2 * _map_height; j++)
//    {
//        for (uint16_t i = 0; i < 2 * _map_width; i++)
//        {
//            if (i < _map_width && j < _map_height)
//            {
//                _extern_disk_grid_map[ j * 2 * _map_width + i] = _disk_grid_map[ j * _map_width + i ];
//            }
//            else
//            {
//                _extern_disk_grid_map[ j * 2 * _map_width + i] = 0;
//            }
//        }
//    }
//    for (uint16_t i = 0; i < _map_size; i++)
//    {
//        _disk_grid_map[i] = 0;
//    }
   
    _disk_grid_map[992] = 100;
    _disk_grid_map[993] = 100;
    _disk_grid_map[960] = 100;
    //_disk_grid_map[961] = 100;
    _disk_grid_map[0] = 100;
    _disk_grid_map[1] = 100;
    _disk_grid_map[32] = 100;
    //_disk_grid_map[33] = 1;
    _disk_grid_map[30] = 100;
    _disk_grid_map[31] = 100;
    //_disk_grid_map[62] = 100;
    _disk_grid_map[63] = 100;
    _disk_grid_map[1022] = 100;
    _disk_grid_map[1023] = 100;
    //_disk_grid_map[990] = 1;
    _disk_grid_map[991] = 100;
    //_disk_grid_map[495] = 1;
    //_disk_grid_map[494] = 1;
    //_disk_grid_map[463] = 1;


//    std::vector<int8_t> map_temp(_disk_grid_map, _disk_grid_map + 400);
//    _disk_occ_map.data = map_temp;

    fft_plan_create();
    ifft_plan_create();

    //fft2d(&_disk_grid_map[0], &_fft_disk_grid_map[0]);
    fft2d(&_disk_grid_map[0], _disk_array);

    //std::vector<int8_t> map_temp(_fft_disk_grid_map, _fft_disk_grid_map + 400);
    //_disk_occ_map.data = map_temp;
//    fft2d();
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
    for (uint16_t i = 0; i < _map_size; i++)
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
            for(uint16_t i = -x - _origin_x; i < (x - _origin_x); i++)
            {
                for (uint16_t j = -y -_origin_y; j < (y - _origin_y); j++)
                {
                    _disk_grid_map[_map_width * j + i] = 100;
                }
            }
            for(uint16_t i = -y - _origin_y; i < (y - _origin_y); i++)
            {
                for (uint16_t j = -x - _origin_x; j < (x - _origin_x); j++)
                {
                    ROS_INFO("2 index x:%d, y:%d", i, j);
                    _disk_grid_map[_map_height * j + i] = 100;
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

int8_t RRT_planner::Planner::fft_plan_create(void)
{
    _d_input_array  = (double*)fftw_malloc(sizeof(double) *  _map_sum_size);
    _c_output_array = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);

//    _fft2d_plan = fftw_plan_dft_2d(_map_width, _map_height, _input_array, _output_array, FFTW_FORWARD, FFTW_MEASURE);
//    _fft2d_plan = fftw_plan_dft_1d(2 * _map_size, _input_array, _output_array, FFTW_FORWARD, FFTW_MEASURE);
//    _fft2d_plan = fftw_plan_dft_r2c_1d(_map_sum_size, _d_input_array, _c_output_array, FFTW_MEASURE);
    _fft2d_plan = fftw_plan_dft_r2c_2d( _map_width,  _map_height, _d_input_array, _c_output_array, FFTW_MEASURE);

    return 0;
}

int8_t RRT_planner::Planner::ifft_plan_create(void)
{
    _c_inverse_input_array  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  _map_sum_size);
    _d_inverse_output_array = (double*)fftw_malloc(sizeof(double) * _map_sum_size);

//    _ifft2d_plan = fftw_plan_dft_2d(_map_width, _map_height,  _inverse_input_array,  _inverse_output_array, FFTW_BACKWARD, FFTW_MEASURE);
//    _ifft2d_plan = fftw_plan_dft_1d(2 * _map_size, _inverse_input_array, _inverse_output_array, FFTW_BACKWARD, FFTW_MEASURE);
//    _ifft2d_plan = fftw_plan_dft_c2r_1d(_map_sum_size, _c_inverse_input_array, _d_inverse_output_array, FFTW_MEASURE);
    _ifft2d_plan = fftw_plan_dft_c2r_2d( _map_width,  _map_height, _c_inverse_input_array, _d_inverse_output_array, FFTW_MEASURE);

    return 0;
}

/*
 * @brief The FFT
 */
//void RRT_planner::Planner::fft2d()
//{
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        _input_array[i][0] = _disk_grid_map[i];
//        _input_array[i][1] = 0;
//    }
//    
//    fftw_execute(_fft2d_plan);
//
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        _fft_disk_grid_map[i] = _output_array[i][0];
//        ROS_INFO("fft grid value:%d",_fft_disk_grid_map[i]);
//    }
//
//    std::vector<int8_t> map_temp(_fft_disk_grid_map, _fft_disk_grid_map + _map_size);
//    _disk_occ_map.data = map_temp;
//}
//
//
//void RRT_planner::Planner::fft2d(int8_t *input_map, int8_t * output_map)
//{
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        _input_array[i][0] = input_map[i];
//        _input_array[i][1] = 0;
//    }
//    
//    fftw_execute(_fft2d_plan);
//
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        output_map[i] = _output_array[i][0];
//        ROS_INFO("fft grid value:%d",output_map[i]);
//    }
//
//}
//
void RRT_planner::Planner::fft2d(int8_t *input_map, fftw_complex *output_map)
{
//    for (uint16_t j = 0; j < 2 * _map_height; j++)
//    {
//        for (uint16_t i = 0; i < 2 * _map_width; i++)
//        {
//            if (i < _map_width && j < _map_height)
//            {
//                _d_input_array[ j * 2 * _map_width + i] = input_map[ j * _map_width + i ];
//            }
//            else
//            {
//                _d_input_array[ j * 2 * _map_width + i] = 0;
//            }
//        }
//    }

    for(uint16_t i=0; i < _map_size; i++)
    {
        ROS_INFO("input  grid value:%d",input_map[i]);
        _d_input_array[i] = input_map[i];
    }
    for(uint16_t i=_map_size; i < _map_sum_size; i++)
    {
        _d_input_array[i] = 0.0;
    }

    fftw_execute(_fft2d_plan);

    for(uint16_t i=0; i <  _map_sum_size; i++)
    {
        output_map[i][0] = _c_output_array[i][0];
        output_map[i][1] = _c_output_array[i][1];
//        ROS_INFO("fft grid value:%f",output_map[i][0]);
    }
}

//void RRT_planner::Planner::ifft2d(int8_t *input_map, fftw_complex * output_map)
//{
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        _inverse_input_array[i][0] = input_map[i];
//        _inverse_input_array[i][1] = 0;
//    }
//    
//    fftw_execute(_ifft2d_plan);
//
//    for(uint16_t i=0; i < _map_size; i++)
//    {
//        output_map[i][0] = _inverse_output_array[i][0];
//        output_map[i][1] = _inverse_output_array[i][1];
//        ROS_INFO("ifft grid value:%f",output_map[i][0]);
//    }
//}   
//
//void RRT_planner::Planner::ifft2d(fftw_complex *input_map, fftw_complex * output_map)
//{
//    for(uint16_t i=0; i < 2 * _map_size; i++)
//    {
//        _inverse_input_array[i][0] = input_map[i][0];
//        _inverse_input_array[i][1] = input_map[i][1];
//    }
//    
//    fftw_execute(_ifft2d_plan);
//
//    for(uint16_t i=0; i < 2 * _map_size; i++)
//    {
//        output_map[i][0] = _inverse_output_array[i][0] ;
//        output_map[i][1] = _inverse_output_array[i][1] ;
//        ROS_INFO("ifft grid value:%f",output_map[i][0]);
//    }
//}

void RRT_planner::Planner::ifft2d(fftw_complex *input_map, int8_t* output_map)
{
     for(uint16_t i=0; i <  _map_sum_size; i++)
    {
        _c_inverse_input_array[i][0] = input_map[i][0];
        _c_inverse_input_array[i][1] = input_map[i][1];
    }
    
    fftw_execute(_ifft2d_plan);

//    for (uint16_t j = 0; j <  _map_height; j++)
//    {
//        for (uint16_t i = 0; i < 2 * _map_width; i++)
//        {
//            if (i < _map_width && j < _map_height)
//            {
//                output_map[ j * _map_width + i] = _d_inverse_output_array[ j * 2 * _map_width + i ] / _map_sum_size > 1.0e-6 ? 100 : 0;
//            }
//        }
//    }

    for(uint16_t i=0; i < _map_sum_size; i++)
    {
        output_map[i] = _d_inverse_output_array[i] / _map_sum_size > 1.0e-6 ? 100 : 0;
//        ROS_INFO("id:%d ifft grid value:%f", i, _d_inverse_output_array[i] / _map_sum_size);
    }
}

/*
 * @brief the callback function for 
 */
void RRT_planner::Planner::MapCallback(const nav_msgs::OccupancyGrid::Ptr map)
{
     _map_height = map->info.height;   
     _map_width  = map->info.width;
    for (uint16_t i = 0; i < _map_height; i++)
    {
       for (uint16_t j = 0; j < _map_width; j++) 
       {
           ROS_INFO("%d ", map->data[i * _map_width + j]);
           _obstacle_grid_map[i * _map_width + j] = map->data[i * _map_width + j] > 0 ? 100 : 0;
       }
    }

//    for (uint16_t i = _map_size; i < 2 * _map_size; i++)
//    {
//        _obstacle_grid_map[i] = 0;
//    }
    //fft2d(&_obstacle_grid_map[0], &_fft_obstacle_grid_map[0]);
    fft2d(&_obstacle_grid_map[0], _obstacle_array);
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

    for (uint16_t i=0; i <  _map_sum_size; i++)
    {
        _c_ifft_input_sum_array[i][0] =  _disk_array[i][0] * _obstacle_array[i][0] - _disk_array[i][1] * _obstacle_array[i][1]; 
        _c_ifft_input_sum_array[i][1] =  _disk_array[i][0] * _obstacle_array[i][1] + _disk_array[i][1] * _obstacle_array[i][0]; 
    }

    ifft2d(_c_ifft_input_sum_array, _ifft_sum_grid_map);
//    ifft2d(_disk_array, _ifft_sum_grid_map);
//    ifft2d(_obstacle_array, _ifft_sum_grid_map);


    std::vector<int8_t> sum_map_temp(_ifft_sum_grid_map , _ifft_sum_grid_map + _map_size);
//    std::vector<int8_t> map_temp(_obstacle_grid_map, _obstacle_grid_map + _map_size);
    _sum_occ_map.data = sum_map_temp;
    sum_grid_map_pub.publish(_sum_occ_map);

    std::vector<int8_t> disk_map_temp(_disk_grid_map, _disk_grid_map +  _map_size);
    _disk_occ_map.data = disk_map_temp;
    disk_grid_map_pub.publish(_disk_occ_map);
}


