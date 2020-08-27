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

        void Init(int8_t *map, uint16_t map_width, uint16_t map_height);

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

        /*
         * @brief create the ifft plan
         */
        int8_t ifft_plan_create(void);

        /*
         * @brief The FFT base on the fftw3 lib
         */
        int8_t fft2d(const int8_t *input_map, fftw_complex * output_map);

        /*
         * @brief The IFFT base on the fftw3 lib
         */
        int8_t ifft2d(const fftw_complex *input_map, int8_t* output_map);

        /*
         * @brief the convolution of two map
         */
        int8_t convolution_2d(const int8_t *input_kernel_map, const int8_t *input_obstacle_map, int8_t *output_sum_map);

        /*
         * @brief the fft base on the fftw3 lib
         * @param input_map: the input map
         * @param input_map_width: the input map width
         * @param input_map_height: the input map height
         * @param output_map: the output frequency obmain map
         * @param output_map_width: the width of output map
         * @param output_map_height: the heigth of the output frequency obmain
         * map
         * @retun the result of fft 
         */
        int8_t fft2d(   const int8_t  *input_map, 
                        const uint16_t input_map_width, 
                        const uint16_t input_map_height, 
                        const uint16_t cov_map_width, 
                        const uint16_t cov_map_height, 
                        fftw_complex  *output_cov_fft_map);

        /*
         * @brief
         */
        int8_t ifft2d(  const fftw_complex *input_fft_map, 
                        const uint16_t cov_map_width, 
                        const uint16_t cov_map_height, 
                        const uint16_t output_map_width, 
                        const uint16_t output_map_height,
                        int8_t *output_map); 

        /*
         * @brief
         */
        int8_t convolution_2d(  const int8_t  *input_kernel_map,   
                                const uint16_t input_kernel_width,   
                                const uint16_t input_kernel_height, 
                                const int8_t  *input_obstacle_map, 
                                const uint16_t input_obstacle_width, 
                                const uint16_t input_obstacle_height, 
                                int8_t *output_sum_map);
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
         * @brief The convolution occupancy map
         */
        nav_msgs::OccupancyGrid _convolution_occ_map;

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
         * @brief The height of fft map
         */
        uint16_t _fft_map_height;

        /*
         * @brief The width of fft map
         */
        uint16_t _fft_map_width;        

        /*
         * @brief The size of fft map
         */
        uint16_t _fft_map_size;

        /*
         * @brief the height of obstacle map 
         */
        uint16_t _obstacle_map_height;

        /*
         * @brief the width of source map
         */
        uint16_t _obstacle_map_width;

        /*
         * @brief the size of obstacle map
         */
        uint16_t _obstacle_map_size;

        /*
         * @brief The origin x axis of obstacle map
         */
        int16_t _obstacle_origin_x;

        /*
         * @brief the flag indicate obstacle map whether update
         */
        bool _is_map_update;

        /*
         * @brief The origin y axis of obstacle map
         */
        int16_t _obstacle_origin_y;

        /*
         * @brief the height of disk map 
         */
        uint16_t _disk_map_height;

        /*
         * @brief the width of disk map
         */
        uint16_t _disk_map_width;

        /*
         * @brief the size of disk map
         */
        uint16_t _disk_map_size;

        /*
         * @brief The origin x axis of disk map
         */
        int16_t _disk_origin_x;

        /*
         * @brief The origin y axis of disk map
         */
        int16_t _disk_origin_y;

        /*
         * @brief The point of disk grid map
         */
        int8_t *_disk_grid_map;

        /*
         * @brief the point of fft disk array
         */
        fftw_complex *_disk_fft_array;

        /*
         * @brief the point of obstacle grid map 
         */
        int8_t *_obstacle_grid_map;

        /*
         * @brief the point of the obstacle fft array
         */
        fftw_complex *_obstacle_fft_array;

        /*
         * @brief the convolution grid map 
         */
        int8_t *_convolution_grid_map;

        /*
         * @brief the frequency domain multiplication result as the input for
         * ifft
         */
        fftw_complex *_c_convolution_ifft_input__array;

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
        fftw_plan _disk_fft2d_plan;

        /*
         * @brief the obstacle fft plan
         */
        fftw_plan _fft2d_plan;

        /*
         * @brief the ifft of the frequency obmain conplex multiplication
         */
        fftw_plan _ifft2d_plan;

    };
}

#endif
