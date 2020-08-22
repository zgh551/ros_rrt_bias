#ifndef _PLANNER_H_
#define _PLANNER_H_

/*
 * @brief The ROS related header file
 */
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
// the msg of start and goal pose
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
// the marker in rviz
#include <visualization_msgs/Marker.h>
// the pose array
#include <geometry_msgs/PoseArray.h>
// the path in rviz
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
/*
 * @brief OMPL lib
 */
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>

/*
 * @brief The State space
 */
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <fftw3.h>
/*
 * @brief the namespace
 */
namespace ob = ompl::base;
namespace og = ompl::geometric;

/*
 * @brief the micro define
 */
#define BOUNDARY_SIZE_X    (32)
#define BOUNDARY_SIZE_Y    (32)
#define BOUNDARY_SIZE      (BOUNDARY_SIZE_X * BOUNDARY_SIZE_Y)

typedef struct _position
{
    double x;
    double y;
    double yaw;
}Position;

namespace RRT_planner
{
    class Planner
    {
        public:
        /*
         * @brief Construct function
         */
        Planner(void);
        ~Planner(void);

        /*
         * @brief Init the planner,such as,the space information, the obstacle
         * checker and the path cost optimization
         */
        void Init(void);

        /*
         * @brief The solve of planner
         */
        void solve(const double time);

        /*
         * @brief Base on the circle draw the rectangle
         */
        void DrawCircle(double r);

        /*
         * @brief create the fft plan
         */
        int8_t fft_plan_create(void);
        int8_t ifft_plan_create(void);

        /*
         * @brief The FFT base on the fftw3 lib
         */
//        void fft2d();
//        void fft2d(int8_t *input_map, int8_t * output_map);
        void fft2d(int8_t *input_map, fftw_complex * output_map);

//        void ifft2d(int8_t *input_map, fftw_complex * output_map);
//        void ifft2d(fftw_complex *input_map, fftw_complex * output_map);
        void ifft2d(fftw_complex *input_map, int8_t* output_map);

        /*
         * @brief The callback function map receive
         */
        void MapCallback(const nav_msgs::OccupancyGrid::Ptr map);

        /*
         * @brief The callback function of start pose
         */
        void StartPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

        /*
         * @brief The callback function of goal pose
         */
        void GoalPoseCallback(const geometry_msgs::PoseStamped &pose);
        private:
        /*
         * @brief the node handle  
         */
        ros::NodeHandle n;

        /*
         * @brief the map subscriber
         */
        ros::Subscriber map_sub;

        /*
         * @brief the subscriber for receive the start update
         */
        ros::Subscriber start_pose_sub;
        /*
         * @brief the subscriber for receive the goal update
         */
        ros::Subscriber goal_pose_sub;

        /*
         * @brief the line marker 
         */
        ros::Publisher line_marker_pub;

        /*
         * @brief the pose array publisher
         */
        ros::Publisher pose_array_pub;

        /*
         * @brief the path to publisher
         */
        ros::Publisher path_state_pub;

        /*
         * @brief The occupancy grid publisher
         */
        ros::Publisher sum_grid_map_pub;
        
        /*
         * @brief the disk map publisher
         */
        ros::Publisher disk_grid_map_pub;

        /*
         * @brief the start position line strip
         */
        visualization_msgs::Marker _start_pose_line_strip;

        /*
         * @brief the goal position line strip
         */
        visualization_msgs::Marker _goal_pose_line_strip;

        /*
         * @brief the rrt sample point show in rviz
         */
        geometry_msgs::PoseArray _sample_point; 

        /*
         * @brief the plan path by the planner
         */
        nav_msgs::Path _plan_path;

        /*
         * @brief The Disk occupancy map
         */
        nav_msgs::OccupancyGrid _disk_occ_map;

        /*
         * @brief The obstacle occupancy map
         */
        nav_msgs::OccupancyGrid _sum_occ_map;

        /*
         * @brief The Space information
         */
        ob::SpaceInformationPtr _si;

        /*
         * @brief The state space
         */
        ob::StateSpacePtr _state_space;
        /*
         * @brief The simple setup pointer
         */
        og::SimpleSetupPtr _ss;

        /*
         * @brief The height of map
         */
        uint16_t _map_height;

        /*
         * @brief The width of map
         */
        uint16_t _map_width;        

        /*
         * @brief The size of map
         */
        uint16_t _map_size;

        uint16_t _map_sum_size;

        /*
         * @brief The origin x axis of map
         */
        int16_t _origin_x;

        /*
         * @brief The origin y axis of map
         */
        int16_t _origin_y;

        /*
         * @brief Disk grid map
         */
        int8_t _disk_grid_map[4 * BOUNDARY_SIZE];
        int8_t _extern_disk_grid_map[4 * BOUNDARY_SIZE];
        int8_t _fft_disk_grid_map[4 * BOUNDARY_SIZE];

        fftw_complex *_disk_array;
        /*
         * @brief Obstacle grid map
         */
        int8_t _obstacle_grid_map[4 * BOUNDARY_SIZE];
        int8_t _fft_obstacle_grid_map[4 * BOUNDARY_SIZE];

        fftw_complex *_obstacle_array;

        /*
         * @brief the sum
         */
        int8_t _fft_sum_grid_map[4 * BOUNDARY_SIZE];

        fftw_complex *_c_ifft_input_sum_array;
        fftw_complex *_ifft_output_sum_array;
        double *_d_ifft_output_sum_array;

        int8_t _ifft_sum_grid_map[4 * BOUNDARY_SIZE];
        /*
         * @brief start position
         */
        Position _start_position;
        bool     _start_valid;

        /*
         * @brief goal position
         */
        Position _goal_position;
        bool     _goal_valid;

        /*
         * @brief The fft input array
         */
        double       *_d_input_array;
        fftw_complex *_c_input_array;
        fftw_complex *_c_inverse_input_array;

        /*
         * @brief The fft output array
         */
        double       *_d_output_array;
        fftw_complex *_c_output_array;
        double       *_d_inverse_output_array;

        /*
         * @brief The fft plan
         */
        fftw_plan _fft2d_plan;
        fftw_plan _ifft2d_plan;

    };
}

#endif
